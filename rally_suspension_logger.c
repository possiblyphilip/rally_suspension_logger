#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/un.h>

#define SAMPLES 5000

#define SENSOR_1 "/dev/ttyUSB0"
#define SENSOR_2 "/dev/ttyUSB1"

#define NUM_MENU_ITEMS 5
#define LCD_LINE_LEN 16

#define MCP_ADDR 0x20
#define IODIRA   0x00
#define GPIOA    0x12
#define GPPUA    0x0C
#define I2C_DEV "/dev/i2c-1"
#define LIVE_AVG_SAMPLES 300
#define SPEED 2000

#define LOG_BUFFER_SIZE 5000  // ~10 seconds at 500Hz



int topout_front = 0;
int topout_rear = 0;
int static_sag_front = -1;
int static_sag_rear = -1;
int race_sag_front = -1;
int race_sag_rear  = -1;


int selected = 0;
int running = 0;
int running_spinner = 0;

volatile int sensor1_dist = -1;
volatile int sensor2_dist = -1;
volatile int sensor1_strength = -1;
volatile int sensor2_strength = -1;

int fd;
static char last_line1[LCD_LINE_LEN + 1] = {0};
static char last_line2[LCD_LINE_LEN + 1] = {0};

char *menu_items[] = {
    "Check Sag",
    "Log",
    "Analyize",
    "Live",
    "Quit"
};

enum Button { NONE, UP, DOWN, LEFT, RIGHT, SELECT };

enum Button button_value = NONE;

typedef struct {
    char lines[LOG_BUFFER_SIZE][64];
    int head;
    int tail;
    pthread_mutex_t lock;
    pthread_cond_t cond;
} log_buffer_t;

log_buffer_t log_buffer = {
    .head = 0,
    .tail = 0,
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .cond = PTHREAD_COND_INITIALIZER
};

void mcp_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    write(fd, buf, 2);
    usleep(100);
}

uint8_t mcp_read(uint8_t reg)
{
    uint8_t data = 0;
    write(fd, &reg, 1);
    read(fd, &data, 1);
    return data;
}

// Button mapping based on Adafruit LCD shield
enum Button read_button()
{
    uint8_t state = mcp_read(GPIOA);

    if(!(state & (1 << 0))) return SELECT;
    if(!(state & (1 << 1))) return RIGHT;
    if(!(state & (1 << 2))) return DOWN;
    if(!(state & (1 << 3))) return UP;
    if(!(state & (1 << 4))) return LEFT;

    return NONE;
}

const char* button_name(enum Button b)
{
    switch(b)
    {
        case SELECT: return "SELECT";
        case RIGHT:  return "RIGHT";
        case DOWN:   return "DOWN";
        case UP:     return "UP";
        case LEFT:   return "LEFT";
        default:     return "NONE";
    }
}

extern enum Button read_button(); // from your button reader
extern const char* button_name(enum Button b);

void truncate_and_escape(const char *input, char *output, size_t max_len)
{
    size_t out_i = 0;
    for(size_t i = 0; i < max_len && input[i] != '\0'; i++)
    {
        if(input[i] == '"' || input[i] == '`' || input[i] == '$' || input[i] == '\\')
        {
            if(out_i < max_len - 1)
                output[out_i++] = '\\'; // escape character
        }
        if(out_i < max_len)
            output[out_i++] = input[i];
    }
    output[out_i] = '\0';
}

void print_to_lcd(const char *line1, const char *line2)
{
    static struct timespec last = {0};
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    double elapsed = (now.tv_sec - last.tv_sec)
                   + (now.tv_nsec - last.tv_nsec) / 1e9;

    if(elapsed < 0.2) // faster refresh for live mode
        return;

    last = now;

    char line1_trim[LCD_LINE_LEN + 1] = {0};
    char line2_trim[LCD_LINE_LEN + 1] = {0};

    strncpy(line1_trim, line1 ? line1 : "", LCD_LINE_LEN);
    strncpy(line2_trim, line2 ? line2 : "", LCD_LINE_LEN);

    // Connect to the LCD server
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if(sock < 0) return;

    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, "/tmp/lcd.sock", sizeof(addr.sun_path) - 1);

    if(connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == 0)
    {
        dprintf(sock, "%s\n%s\n", line1_trim, line2_trim);
    }

    close(sock);
}

typedef struct {
    char prefix[33]; // or whatever size you need
} spinner_args_t;

void *lcd_spinner_thread(void *arg)
{
    spinner_args_t *args = (spinner_args_t *)arg;
    const char *prefix = args->prefix;
    const char spin[] = "|/-*";
    int frame = 0;

    while(running_spinner)
    {
        char line1[17];
        snprintf(line1, sizeof(line1), "%-15s%c", prefix, spin[frame % 4]);

        print_to_lcd(line1, NULL);

        printf("\r%s %c", prefix, spin[frame % 4]);
        fflush(stdout);

        frame++;
        usleep(750000); 
    }

    return NULL;
}

void draw_menu()
{
    int start = (selected == 0 || selected % 2 == 0) ? selected : selected - 1;

    char line1[LCD_LINE_LEN + 1] = {0};
    char line2[LCD_LINE_LEN + 1] = {0};

    if(start < NUM_MENU_ITEMS)
        snprintf(line1, LCD_LINE_LEN + 1, "%c%-15s", selected == start ? '>' : ' ', menu_items[start]);
    if(start + 1 < NUM_MENU_ITEMS)
        snprintf(line2, LCD_LINE_LEN + 1, "%c%-15s", selected == start + 1 ? '>' : ' ', menu_items[start + 1]);

	sleep(1);
    print_to_lcd(line1, line2);
}

void tfmini_setup(int fd)
{
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;
    tcsetattr(fd, TCSANOW, &tty);

    uint8_t cmd[] = {0x5A, 0x05, 0x00, 0x01, 0x60};
    write(fd, cmd, sizeof(cmd));
    usleep(100000); // match what worked
}

void *read_sensor(void *arg)
{
    char *dev = (char *)arg;
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0)
    {
        perror("open sensor");
        return NULL;
    }

    tcflush(fd, TCIFLUSH); // clear junk

    unsigned char buf[9];
    int state = 0;

    while(running)
    {
        unsigned char byte;
        int n = read(fd, &byte, 1);
        if(n == 1)
        {
            switch(state)
            {
                case 0:
                    if(byte == 0x59)
                        state = 1;
                    break;
                case 1:
                    if(byte == 0x59)
                    {
                        buf[0] = 0x59;
                        buf[1] = 0x59;
                        state = 2;
                    }
                    else
                        state = 0;
                    break;
                default:
                    buf[state++] = byte;
                    if(state == 9)
                    {
                        int dist = buf[2] + (buf[3] << 8);
                        int strength = buf[4] + (buf[5] << 8);
                        if(dist > 0 && dist < 1200)
                        {
                            if(strcmp(dev, SENSOR_1) == 0)
                            {
                                sensor1_dist = dist;
                                sensor1_strength = strength;
                            }
                            else
                            {
                                sensor2_dist = dist;
                                sensor2_strength = strength;
                            }
                        }
                        state = 0;
                    }
                    break;
            }
        }
        else
        {
            usleep(SPEED);
        }
    }

    close(fd);
    return NULL;
}


void *log_writer_thread(void *arg)
{
    FILE *logfile = (FILE *)arg;

    // Buffered output
    char *logbuf = malloc(65536);
    if(logbuf)
        setvbuf(logfile, logbuf, _IOFBF, 65536);

    while(running || log_buffer.head != log_buffer.tail)
    {
        pthread_mutex_lock(&log_buffer.lock);
        while(log_buffer.head == log_buffer.tail && running)
            pthread_cond_wait(&log_buffer.cond, &log_buffer.lock);

        while(log_buffer.head != log_buffer.tail)
        {
            fputs(log_buffer.lines[log_buffer.tail], logfile);
            log_buffer.tail = (log_buffer.tail + 1) % LOG_BUFFER_SIZE;
        }

        pthread_mutex_unlock(&log_buffer.lock);
        fflush(logfile);
    }

    // Final flush
    fflush(logfile);
    if(logbuf)
        free(logbuf);
    return NULL;
}

void *log_thread(void *arg)
{
    pthread_t t1, t2, writer_thread, spinner_thread;
    running = 1;

    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char filename[64];
    strftime(filename, sizeof(filename), "log_%Y-%m-%dT%H-%M-%S.csv", tm_info);

    FILE *logfile = fopen(filename, "w");
    if(!logfile)
    {
        print_to_lcd("Failed to", "open log");
        perror("Failed to open log file");
        running = 0;
        return NULL;
    }

    fputs("time_s,sensor1_cm,sensor2_cm\n", logfile);

    pthread_create(&t1, NULL, read_sensor, (void *)SENSOR_1);
    pthread_create(&t2, NULL, read_sensor, (void *)SENSOR_2);

    while(sensor1_dist == -1 || sensor2_dist == -1)
        usleep(SPEED);

    print_to_lcd("Logging to", filename);
    sleep(2);

    static spinner_args_t spinner_data = {.prefix = "Logging..."};
    running_spinner = 1;
    pthread_create(&spinner_thread, NULL, lcd_spinner_thread, &spinner_data);
    pthread_create(&writer_thread, NULL, log_writer_thread, logfile);

    struct timespec start, now_time, delay = {0, SPEED * 1000};
    clock_gettime(CLOCK_MONOTONIC, &start);

    while(running)
    {
        clock_gettime(CLOCK_MONOTONIC, &now_time);
        double elapsed = (now_time.tv_sec - start.tv_sec)
                       + (now_time.tv_nsec - start.tv_nsec) / 1e9;

        char line[64];
        snprintf(line, sizeof(line), "%.6f,%d,%d\n", elapsed, sensor1_dist, sensor2_dist);

        pthread_mutex_lock(&log_buffer.lock);
        int next_head = (log_buffer.head + 1) % LOG_BUFFER_SIZE;
        if(next_head != log_buffer.tail)
        {
            strcpy(log_buffer.lines[log_buffer.head], line);
            log_buffer.head = next_head;
            pthread_cond_signal(&log_buffer.cond);
        }
        pthread_mutex_unlock(&log_buffer.lock);

        nanosleep(&delay, NULL);
    }
    
    print_to_lcd("Saving to", filename);

	running = 0;
	pthread_cond_signal(&log_buffer.cond);
	
	pthread_join(t1, NULL);
	pthread_join(t2, NULL);
	running_spinner = 0;
	pthread_join(spinner_thread, NULL);
	pthread_join(writer_thread, NULL);
	fclose(logfile);


    
    return NULL;
}

void *live_thread(void *arg)
{
    pthread_t t1, t2;
    running = 1;
    pthread_create(&t1, NULL, read_sensor, (void *)SENSOR_1);
    pthread_create(&t2, NULL, read_sensor, (void *)SENSOR_2);

    printf("LIVE MODE - Press LEFT to return to menu\n");

    char line1[17], line2[17];
    int s1_history[LIVE_AVG_SAMPLES] = {0};
    int s2_history[LIVE_AVG_SAMPLES] = {0};
    int index = 0;
    int initialized = 0;

    struct timespec last = {0};

    enum Button last_button = NONE;

    while(running)
    {
        int cur1 = sensor1_dist * 10;
        int cur2 = sensor2_dist * 10;

        if(!initialized && cur1 > 0 && cur2 > 0)
        {
            for(int i = 0; i < LIVE_AVG_SAMPLES; i++)
            {
                s1_history[i] = cur1;
                s2_history[i] = cur2;
            }
            initialized = 1;
            clock_gettime(CLOCK_MONOTONIC, &last);
        }

        s1_history[index] = cur1;
        s2_history[index] = cur2;
        index = (index + 1) % LIVE_AVG_SAMPLES;

        enum Button b = read_button();
        if(b != last_button)
        {
            if(b == DOWN)
            {
                int sum1 = 0, sum2 = 0;
                for(int i = 0; i < LIVE_AVG_SAMPLES; i++)
                {
                    sum1 += s1_history[i];
                    sum2 += s2_history[i];
                }

                topout_front = sum1 / LIVE_AVG_SAMPLES;
                topout_rear  = sum2 / LIVE_AVG_SAMPLES;
                sleep(1);

                print_to_lcd("Zeroed", "Topout set");
                sleep(2);
            }
            else if(b == UP)
            {
                topout_front = 0;
                topout_rear = 0;
                sleep(1);
                print_to_lcd("Reset", "Topout cleared");
                sleep(2);
            }

            last_button = b;
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        double elapsed = (now.tv_sec - last.tv_sec)
                       + (now.tv_nsec - last.tv_nsec) / 1e9;

        if(elapsed >= 0.75)
        {
            last = now;

            int sum1 = 0, sum2 = 0;
            for(int i = 0; i < LIVE_AVG_SAMPLES; i++)
            {
                sum1 += s1_history[i];
                sum2 += s2_history[i];
            }

            int avg1 = sum1 / LIVE_AVG_SAMPLES;
            int avg2 = sum2 / LIVE_AVG_SAMPLES;

            snprintf(line1, sizeof(line1), "F:%3d R:%3d mm", avg1 - topout_front, avg2 - topout_rear);
            snprintf(line2, sizeof(line2), "S1:%3d  S2:%3d", sensor1_strength/600, sensor2_strength/600);

            printf("\033[2J\033[H%-16s\n%-16s\n", line1, line2);
            print_to_lcd(line1, line2);
        }

        usleep(2000);
    }

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    printf("\n");
    return NULL;
}

void *check_sag_thread(void *arg)
{
	int ii;
	char line1[17];
	char line2[17];
    int max_front = 0, max_rear = 0;
    
    int cur1 = 0;
    int cur2 = 0;
    
    int s1_history[SAMPLES] = {0};
    int s2_history[SAMPLES] = {0};
    running = 1;
    pthread_t t1, t2, spinner_thread;
    pthread_create(&t1, NULL, read_sensor, (void *)SENSOR_1);
    pthread_create(&t2, NULL, read_sensor, (void *)SENSOR_2);
    
    // Optionally flush both ports
	int fd1 = open(SENSOR_1, O_RDWR | O_NOCTTY);
	int fd2 = open(SENSOR_2, O_RDWR | O_NOCTTY);
	if(fd1 >= 0) { tcflush(fd1, TCIFLUSH); close(fd1); }
	if(fd2 >= 0) { tcflush(fd2, TCIFLUSH); close(fd2); }


    print_to_lcd("Set on stand", "press select");
    
    while(read_button() != SELECT)
	{
	    usleep(100000);
	}
	
	spinner_args_t spinner_data;
	strcpy(spinner_data.prefix, "Calibrating...");

    running_spinner = 1;
    pthread_create(&spinner_thread, NULL, lcd_spinner_thread, &spinner_data);
    sleep(1);
    
	while(running && (sensor1_dist == -1 || sensor2_dist == -1))
	{
	    usleep(1000);
	}

//get top out
   	ii = 0;
	while(running && ii < SAMPLES)
    {
   		s1_history[ii] = sensor1_dist * 10;
        s2_history[ii] = sensor2_dist * 10;
		usleep(2000); // ~500 Hz loop rate
		ii++;
    }
    
    int sum1 = 0, sum2 = 0;
    for(int i = 0; i < SAMPLES; i++)
    {
    	sum1 += s1_history[i];
     	sum2 += s2_history[i];
    }

    topout_front = sum1 / SAMPLES;
    topout_rear = sum2 / SAMPLES;
    
    running_spinner = 0;
    pthread_join(spinner_thread, NULL);
    sleep(1);


    print_to_lcd("Set on wheels", "press select");
    while(read_button() != SELECT)
	{
	    usleep(100000);
	}
	
	strcpy(spinner_data.prefix, "static sag ");
    running_spinner = 1;
    pthread_create(&spinner_thread, NULL, lcd_spinner_thread, &spinner_data);

//get static sag
	ii = 0;
	while(running && ii < SAMPLES)
    {
   		s1_history[ii] = sensor1_dist * 10;
        s2_history[ii] = sensor2_dist * 10;
		usleep(2000); // ~500 Hz loop rate
		ii++;
    }
    
    sum1 = 0;
    sum2 = 0;
    for(int i = 0; i < SAMPLES; i++)
    {
    	sum1 += s1_history[i];
     	sum2 += s2_history[i];
    }

    static_sag_front = topout_front - (sum1 / SAMPLES);
    static_sag_rear = topout_rear - (sum2 / SAMPLES);
 
    running_spinner = 0;
    pthread_join(spinner_thread, NULL);   
    sleep(1);
    
    snprintf(line1, sizeof(line1), "F:%3d R:%3d mm", static_sag_front, static_sag_rear);
    print_to_lcd(line1, "press select");
    while(read_button() != SELECT)
	{
	    usleep(100000);
	}
	sleep(1);
    
//get race sag
    print_to_lcd("Sit on bike", "press select");
    while(read_button() != SELECT)
	{
	    usleep(100000);
	}
	
	strcpy(spinner_data.prefix, "race sag ");
    running_spinner = 1;
    pthread_create(&spinner_thread, NULL, lcd_spinner_thread, &spinner_data);

	ii = 0;
	while(running && ii < SAMPLES)
    {
   		s1_history[ii] = sensor1_dist * 10;
        s2_history[ii] = sensor2_dist * 10;
		usleep(2000); // ~500 Hz loop rate
		ii++;
    }
    
    sum1 = 0;
    sum2 = 0;
    for(int i = 0; i < SAMPLES; i++)
    {
    	sum1 += s1_history[i];
     	sum2 += s2_history[i];
    }

    race_sag_front = topout_front - (sum1 / SAMPLES);
    race_sag_rear = topout_rear -(sum2 / SAMPLES);
 
    running_spinner = 0;
    pthread_join(spinner_thread, NULL); 
    sleep(1); 
    
    snprintf(line1, sizeof(line1), "SF:%3d R:%3d mm", static_sag_front, static_sag_rear ); 
    snprintf(line2, sizeof(line2), "RF:%3d R:%3d mm", race_sag_front, race_sag_rear); 

    print_to_lcd(line1, line2);
    
    while(read_button() != SELECT)
	{
	    usleep(100000);
	}

    running = 0;
	usleep(100000);
	
    pthread_join(t1, NULL);
    pthread_join(t2, NULL);

    return NULL;
}

void *analize_thread(void *arg)
{
	return NULL;
}

void run_menu()
{
    pthread_t thread_id;
    enum Button last = NONE;
    struct timespec delay = {0, 100 * 1000000}; // 100ms debounce

    draw_menu();

    while(1)
    {
        button_value = read_button();
        if(button_value != NONE && button_value != last)
        {
            last = button_value;
            switch(button_value)
            {
                case UP:
                    if(selected > 0) selected--;
                    draw_menu();
                    break;
                case DOWN:
                    if(selected < NUM_MENU_ITEMS - 1) selected++;
                    draw_menu();
                    break;
                case RIGHT:
                    if(strcmp(menu_items[selected], "Live") == 0)
                    {
                        pthread_create(&thread_id, NULL, live_thread, NULL);
                    }
                    else if(strcmp(menu_items[selected], "Log") == 0)
                    {
                        pthread_create(&thread_id, NULL, log_thread, NULL);			
                    }
                    else if(strcmp(menu_items[selected], "Analize") == 0)
                    {
                        pthread_create(&thread_id, NULL, analize_thread, NULL);			
                    }
                    
                    else if(strcmp(menu_items[selected], "Check Sag") == 0)
                    {
                        pthread_create(&thread_id, NULL, check_sag_thread, NULL);
                    }                         
                    else if(strcmp(menu_items[selected], "Quit") == 0)
                    {
                        if(running)
                        {
                            running = 0;
                            pthread_join(thread_id, NULL);
                        }
                        print_to_lcd("Safe To", "Power Down");

                        return;
                    }
                    break;
                case LEFT:
                    if(running)
                    {
                        running = 0;
                        sleep(2); //give stuff time to exit
                        pthread_join(thread_id, NULL);
                        sleep(1);
                    }     
                    draw_menu();
                    break;
                case SELECT:
                    // Optional: add select logic here
                    break;
                default:
                    break;
            }
        }
        else if(button_value == NONE)
        {
            last = NONE; // allow next press to be detected
        }

        nanosleep(&delay, NULL); // debounce wait
    }
}

int main()
{
	
	mcp_write(IODIRA, 0xDF); // 1101_1111 — inputs A0–A4, output A6
    mcp_write(GPPUA, 0x1F);   // Pullups on A0–A4
    
	sleep(2);
	
    print_to_lcd("Welcome To", NULL);
    sleep(2);
    print_to_lcd("Rally Suspension", "Logger");
    sleep(2);
    
    print_to_lcd("Loading", "Sensors");
	int fd1 = open(SENSOR_1, O_RDWR | O_NOCTTY);
	if(fd1 >= 0)
	{
	    tfmini_setup(fd1);
	    close(fd1);
	}
	
//	print_to_lcd(NULL, "Loading Rear");
	int fd2 = open(SENSOR_2, O_RDWR | O_NOCTTY);
	if(fd2 >= 0)
	{
	    tfmini_setup(fd2);
	    close(fd2);
	}

    fd = open(I2C_DEV, O_RDWR);
    if(fd < 0 || ioctl(fd, I2C_SLAVE, MCP_ADDR) < 0)
    {
        perror("Failed to init I2C for buttons");
        return 1;
    }




    run_menu();
    
    print_to_lcd("Go", "Ride");
    return 0;
}
