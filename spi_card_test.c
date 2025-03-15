#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

/* Define ioctl commands - must match driver */
#define ROV_SPI_IOC_MAGIC 'r'
#define ROV_SPI_IOC_SELECT_CARD     _IOW(ROV_SPI_IOC_MAGIC, 1, int)
#define ROV_SPI_IOC_GET_CURRENT_CARD _IOR(ROV_SPI_IOC_MAGIC, 2, int)
#define ROV_SPI_IOC_SET_GPIO_DIRECT  _IOW(ROV_SPI_IOC_MAGIC, 3, unsigned int)
#define ROV_SPI_IOC_GET_GPIO_STATE   _IOR(ROV_SPI_IOC_MAGIC, 4, unsigned int)

#define DEVICE_PATH "/dev/rov_eeprom"

void print_usage(const char *prog_name) {
    printf("Usage: %s <command> [args]\n", prog_name);
    printf("Commands:\n");
    printf("  select <card_num>    - Select a card by number (0-15)\n");
    printf("  current              - Get current card number\n");
    printf("  gpio_set <pattern>   - Set GPIO pattern directly (hex value)\n");
    printf("  gpio_get             - Get current GPIO state\n");
    printf("  read <offset> <len>  - Read data from EEPROM\n");
    printf("  write <offset> <data>- Write string to EEPROM\n");
    printf("Examples:\n");
    printf("  %s select 12         - Select card 12\n", prog_name);
    printf("  %s gpio_set 0xC      - Set GPIO pattern to 1100 (card 12)\n", prog_name);
    printf("  %s read 0 16         - Read 16 bytes from offset 0\n", prog_name);
}

int select_card(int fd, int card) {
    if (ioctl(fd, ROV_SPI_IOC_SELECT_CARD, &card) < 0) {
        perror("Failed to select card");
        return -1;
    }
    printf("Card %d selected successfully\n", card);
    return 0;
}

int get_current_card(int fd) {
    int card;
    if (ioctl(fd, ROV_SPI_IOC_GET_CURRENT_CARD, &card) < 0) {
        perror("Failed to get current card");
        return -1;
    }
    printf("Current card: %d\n", card);
    return 0;
}

int set_gpio_pattern(int fd, unsigned int pattern) {
    if (ioctl(fd, ROV_SPI_IOC_SET_GPIO_DIRECT, &pattern) < 0) {
        perror("Failed to set GPIO pattern");
        return -1;
    }
    printf("GPIO pattern set to 0x%x\n", pattern);
    return 0;
}

int get_gpio_state(int fd) {
    unsigned int pattern;
    if (ioctl(fd, ROV_SPI_IOC_GET_GPIO_STATE, &pattern) < 0) {
        perror("Failed to get GPIO state");
        return -1;
    }
    printf("Current GPIO pattern: 0x%x (binary: ", pattern);
    
    /* Print binary representation */
    for (int i = 3; i >= 0; i--) {
        printf("%d", (pattern >> i) & 1);
    }
    printf(")\n");
    
    return 0;
}

int read_eeprom(int fd, off_t offset, size_t length) {
    unsigned char *buffer = malloc(length);
    if (!buffer) {
        perror("Failed to allocate memory");
        return -1;
    }
    
    if (lseek(fd, offset, SEEK_SET) != offset) {
        perror("Failed to set file position");
        free(buffer);
        return -1;
    }
    
    ssize_t bytes_read = read(fd, buffer, length);
    if (bytes_read < 0) {
        perror("Read failed");
        free(buffer);
        return -1;
    }
    
    printf("Read %zd bytes from offset %lld:\n", bytes_read, (long long)offset);
    
    /* Print hex dump */
    for (ssize_t i = 0; i < bytes_read; i++) {
        printf("%02X ", buffer[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
    
    /* Print ASCII representation if it looks like text */
    int printable = 1;
    for (ssize_t i = 0; i < bytes_read && printable; i++) {
        if (buffer[i] < 32 || buffer[i] > 126) {
            if (buffer[i] != 0 && buffer[i] != '\n' && buffer[i] != '\r') {
                printable = 0;
            }
        }
    }
    
    if (printable) {
        printf("ASCII: ");
        for (ssize_t i = 0; i < bytes_read; i++) {
            if (buffer[i] >= 32 && buffer[i] <= 126) {
                printf("%c", buffer[i]);
            } else {
                printf(".");
            }
        }
        printf("\n");
    }
    
    free(buffer);
    return 0;
}

int write_eeprom(int fd, off_t offset, const char *data) {
    size_t length = strlen(data);
    
    if (lseek(fd, offset, SEEK_SET) != offset) {
        perror("Failed to set file position");
        return -1;
    }
    
    ssize_t bytes_written = write(fd, data, length);
    if (bytes_written < 0) {
        perror("Write failed");
        return -1;
    }
    
    printf("Wrote %zd bytes to offset %lld\n", bytes_written, (long long)offset);
    return 0;
}

int main(int argc, char **argv) {
    int fd;
    
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }
    
    if (strcmp(argv[1], "select") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Error: select command requires card number\n");
            close(fd);
            return 1;
        }
        
        int card = atoi(argv[2]);
        select_card(fd, card);
    }
    else if (strcmp(argv[1], "current") == 0) {
        get_current_card(fd);
    }
    else if (strcmp(argv[1], "gpio_set") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Error: gpio_set command requires pattern\n");
            close(fd);
            return 1;
        }
        
        unsigned int pattern;
        sscanf(argv[2], "%x", &pattern);
        set_gpio_pattern(fd, pattern);
    }
    else if (strcmp(argv[1], "gpio_get") == 0) {
        get_gpio_state(fd);
    }
    else if (strcmp(argv[1], "read") == 0) {
        if (argc < 4) {
            fprintf(stderr, "Error: read command requires offset and length\n");
            close(fd);
            return 1;
        }
        
        off_t offset = atol(argv[2]);
        size_t length = atoi(argv[3]);
        read_eeprom(fd, offset, length);
    }
    else if (strcmp(argv[1], "write") == 0) {
        if (argc < 4) {
            fprintf(stderr, "Error: write command requires offset and data\n");
            close(fd);
            return 1;
        }
        
        off_t offset = atol(argv[2]);
        write_eeprom(fd, offset, argv[3]);
    }
    else {
        fprintf(stderr, "Unknown command: %s\n", argv[1]);
        print_usage(argv[0]);
        close(fd);
        return 1;
    }
    
    close(fd);
    return 0;
}
