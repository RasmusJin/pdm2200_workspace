/**
 * rov_eeprom_flash.c - EEPROM Flashing Utility for ROV 2200 System
 *
 * Copyright (C) 2025 Rovsing A/S
 * Author: Rasmus Jin Mayland (RJM)
 *
 * This program provides a user-friendly interface for system engineers 
 * to program daughterboard EEPROMs before shipping.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>

/* EEPROM memory map offsets */
#define EEPROM_OFFSET_CARD_ID         0x00  /* 6 bytes */
#define EEPROM_OFFSET_REVISION        0x06  /* 2 bytes */
#define EEPROM_OFFSET_SERIAL_NUM      0x08  /* 2 bytes */
#define EEPROM_OFFSET_CAL_YEAR        0x0A  /* 2 bytes */
#define EEPROM_OFFSET_CAL_MONTH       0x0C  /* 2 bytes */
#define EEPROM_OFFSET_GAIN_LINEARITY  0x0E  /* 8 bytes */
#define EEPROM_OFFSET_OFFSET_DATA     0x16  /* 8 bytes */
#define EEPROM_OFFSET_COMMAND_ID      0x1E  /* 2 bytes */

/* EEPROM total size */
#define EEPROM_SIZE 256

/* IOCTL commands - must match driver definitions */
#define ROV_SPI_IOC_MAGIC 'r'
#define ROV_SPI_IOC_SELECT_CARD     _IOW(ROV_SPI_IOC_MAGIC, 1, int)
#define ROV_SPI_IOC_GET_CURRENT_CARD _IOR(ROV_SPI_IOC_MAGIC, 2, int)

/* EEPROM device path */
#define EEPROM_DEVICE "/dev/rov_eeprom"

/* Buffer to hold the complete EEPROM image */
static uint8_t eeprom_data[EEPROM_SIZE];

/* File descriptor for the EEPROM device */
static int eeprom_fd = -1;

/* Currently selected card */
static int current_card = -1;

/* Function prototypes */
static void show_main_menu(void);
static void select_card(void);
static void program_eeprom(void);
static void read_eeprom(void);
static void dump_eeprom_data(uint8_t *data, size_t length);
static bool get_confirmation(const char *prompt);
static void clean_stdin(void);
static bool validate_hex_string(const char *str, size_t expected_len);
static bool validate_number(const char *str, int min, int max);
static bool hex_string_to_bytes(const char *hex, uint8_t *bytes, size_t len);

int main(int argc, char *argv[])
{
    int choice;
    bool running = true;

    printf("\n====================================================\n");
    printf("  ROV 2200 EEPROM Flashing Utility v1.0\n");
    printf("====================================================\n\n");

    /* Open the EEPROM device */
    eeprom_fd = open(EEPROM_DEVICE, O_RDWR);
    if (eeprom_fd < 0) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", 
                EEPROM_DEVICE, strerror(errno));
        return EXIT_FAILURE;
    }

    /* Initialize EEPROM data buffer to 0xFF (erased state) */
    memset(eeprom_data, 0xFF, sizeof(eeprom_data));

    /* Main program loop */
    while (running) {
        show_main_menu();
        
        printf("Enter your choice: ");
        if (scanf("%d", &choice) != 1) {
            clean_stdin();
            printf("Invalid input. Please enter a number.\n");
            continue;
        }
        clean_stdin();
        
        switch (choice) {
            case 1:
                select_card();
                break;
            case 2:
                program_eeprom();
                break;
            case 3:
                read_eeprom();
                break;
            case 4:
                printf("Exiting program. Goodbye!\n");
                running = false;
                break;
            default:
                printf("Invalid choice. Please try again.\n");
                break;
        }
        
        printf("\nPress Enter to continue...");
        getchar();
    }

    /* Close the EEPROM device */
    if (eeprom_fd >= 0) {
        close(eeprom_fd);
    }

    return EXIT_SUCCESS;
}

static void show_main_menu(void)
{
    printf("\n====================================================\n");
    printf("  ROV 2200 EEPROM Flashing Utility - Main Menu\n");
    printf("====================================================\n");
    printf("Currently selected card: %d\n", current_card);
    printf("\n");
    printf("1. Select Card\n");
    printf("2. Program EEPROM\n");
    printf("3. Read EEPROM\n");
    printf("4. Exit\n");
    printf("====================================================\n");
}

static void select_card(void)
{
    int card;
    
    printf("\n====================================================\n");
    printf("  Card Selection\n");
    printf("====================================================\n");
    printf("Available cards: 0-15 (Card 12 is default)\n");
    printf("Enter card number: ");
    
    if (scanf("%d", &card) != 1) {
        clean_stdin();
        printf("Invalid input. Please enter a number.\n");
        return;
    }
    clean_stdin();
    
    if (card < 0 || card > 15) {
        printf("Error: Card number must be between 0 and 15.\n");
        return;
    }
    
    /* Select the card using ioctl */
    if (ioctl(eeprom_fd, ROV_SPI_IOC_SELECT_CARD, &card) < 0) {
        fprintf(stderr, "Error: Failed to select card %d: %s\n", 
                card, strerror(errno));
        return;
    }
    
    current_card = card;
    printf("Card %d selected successfully.\n", card);
}

static void program_eeprom(void)
{
    char input[128];
    char *endptr;
    uint16_t value;
    bool valid_input = false;
    
    printf("\n====================================================\n");
    printf("  Program EEPROM\n");
    printf("====================================================\n");
    
    if (current_card < 0) {
        printf("Error: No card selected. Please select a card first.\n");
        return;
    }
    
    /* Initialize EEPROM data to 0xFF (erased state) */
    memset(eeprom_data, 0xFF, sizeof(eeprom_data));
    
    /* 1. CARD ID (6 decimal digits) */
    while (!valid_input) {
        printf("Enter Card ID (6 decimal digits): ");
        if (!fgets(input, sizeof(input), stdin)) {
            continue;
        }
        input[strcspn(input, "\n")] = 0;
        
        /* Validate: exactly 6 digits, no other characters */
        if (strlen(input) != 6) {
            printf("Error: Card ID must be exactly 6 digits.\n");
            continue;
        }
        
        valid_input = true;
        for (int i = 0; i < 6; i++) {
            if (!isdigit((unsigned char)input[i])) {
                printf("Error: Card ID must contain only digits (0-9).\n");
                valid_input = false;
                break;
            }
        }
        
        if (valid_input) {
            /* Store as 3 words (2 digits per word) */
            for (int i = 0; i < 3; i++) {
                char word_str[3] = {input[i*2], input[i*2+1], 0};
                value = (uint16_t)strtol(word_str, NULL, 10);
                eeprom_data[EEPROM_OFFSET_CARD_ID + i*2] = (value >> 8) & 0xFF;
                eeprom_data[EEPROM_OFFSET_CARD_ID + i*2 + 1] = value & 0xFF;
            }
        }
    }
    
    /* 2. REVISION (letter followed by number, e.g., A9) */
    valid_input = false;
    while (!valid_input) {
        printf("Enter Revision (letter followed by number, e.g., A9): ");
        if (!fgets(input, sizeof(input), stdin)) {
            continue;
        }
        input[strcspn(input, "\n")] = 0;
        
        /* Validate: exactly 2 characters - letter followed by digit */
        if (strlen(input) != 2) {
            printf("Error: Revision must be exactly 2 characters (letter followed by digit).\n");
            continue;
        }
        
        if (!isalpha((unsigned char)input[0]) || !isdigit((unsigned char)input[1])) {
            printf("Error: Revision must be a letter followed by a digit.\n");
            continue;
        }
        
        /* Auto-capitalize the letter */
        input[0] = toupper((unsigned char)input[0]);
        valid_input = true;
        
        /* Store as 1 word (2 ASCII characters) */
        eeprom_data[EEPROM_OFFSET_REVISION] = input[0];
        eeprom_data[EEPROM_OFFSET_REVISION + 1] = input[1];
    }
    
    /* 3. SERIAL NUMBER (numeric) */
    valid_input = false;
    while (!valid_input) {
        printf("Enter Serial Number (0-65535): ");
        if (!fgets(input, sizeof(input), stdin)) {
            continue;
        }
        input[strcspn(input, "\n")] = 0;
        
        /* Validate: numeric value within range */
        for (size_t i = 0; i < strlen(input); i++) {
            if (!isdigit((unsigned char)input[i])) {
                printf("Error: Serial Number must contain only digits.\n");
                break;
            }
            
            if (i == strlen(input) - 1) {
                valid_input = true;
            }
        }
        
        if (valid_input) {
            value = (uint16_t)strtol(input, &endptr, 10);
            if (*endptr != '\0' || value > 65535) {
                printf("Error: Invalid Serial Number. Must be 0-65535.\n");
                valid_input = false;
                continue;
            }
            
            /* Store as MSB/LSB */
            eeprom_data[EEPROM_OFFSET_SERIAL_NUM] = (value >> 8) & 0xFF;
            eeprom_data[EEPROM_OFFSET_SERIAL_NUM + 1] = value & 0xFF;
        }
    }
    
    /* 4. CALIBRATION YEAR */
    valid_input = false;
    while (!valid_input) {
        printf("Enter Calibration Year (e.g., 2024): ");
        if (!fgets(input, sizeof(input), stdin)) {
            continue;
        }
        input[strcspn(input, "\n")] = 0;
        
        /* Validate: numeric value within reasonable range */
        for (size_t i = 0; i < strlen(input); i++) {
            if (!isdigit((unsigned char)input[i])) {
                printf("Error: Calibration Year must contain only digits.\n");
                break;
            }
            
            if (i == strlen(input) - 1) {
                valid_input = true;
            }
        }
        
        if (valid_input) {
            value = (uint16_t)strtol(input, &endptr, 10);
            if (*endptr != '\0' || value < 2000 || value > 2100) {
                printf("Error: Invalid Calibration Year. Must be between 2000-2100.\n");
                valid_input = false;
                continue;
            }
            
            /* Store as 1 word */
            eeprom_data[EEPROM_OFFSET_CAL_YEAR] = (value >> 8) & 0xFF;
            eeprom_data[EEPROM_OFFSET_CAL_YEAR + 1] = value & 0xFF;
        }
    }
    
    /* 5. CALIBRATION MONTH */
    valid_input = false;
    while (!valid_input) {
        printf("Enter Calibration Month (1-12): ");
        if (!fgets(input, sizeof(input), stdin)) {
            continue;
        }
        input[strcspn(input, "\n")] = 0;
        
        /* Validate: numeric value 1-12 */
        for (size_t i = 0; i < strlen(input); i++) {
            if (!isdigit((unsigned char)input[i])) {
                printf("Error: Calibration Month must contain only digits.\n");
                break;
            }
            
            if (i == strlen(input) - 1) {
                valid_input = true;
            }
        }
        
        if (valid_input) {
            value = (uint16_t)strtol(input, &endptr, 10);
            if (*endptr != '\0' || value < 1 || value > 12) {
                printf("Error: Invalid Calibration Month. Must be 1-12.\n");
                valid_input = false;
                continue;
            }
            
            /* Store as 1 word */
            eeprom_data[EEPROM_OFFSET_CAL_MONTH] = (value >> 8) & 0xFF;
            eeprom_data[EEPROM_OFFSET_CAL_MONTH + 1] = value & 0xFF;
        }
    }
    
    /* 6. GAIN LINEARITY DATA - Default to zeros for now */
    printf("Setting Gain Linearity Data to default (all zeros)...\n");
    for (int i = 0; i < 8; i++) {
        eeprom_data[EEPROM_OFFSET_GAIN_LINEARITY + i] = 0x00;
    }
    
    /* 7. OFFSET DATA - Default to zeros for now */
    printf("Setting Offset Data to default (all zeros)...\n");
    for (int i = 0; i < 8; i++) {
        eeprom_data[EEPROM_OFFSET_OFFSET_DATA + i] = 0x00;
    }
    
    /* 8. COMMAND ID */
    valid_input = false;
    while (!valid_input) {
        printf("Enter Command ID (decimal, 0-65535): ");
        if (!fgets(input, sizeof(input), stdin)) {
            continue;
        }
        input[strcspn(input, "\n")] = 0;
        
        /* Validate: numeric value within range */
        for (size_t i = 0; i < strlen(input); i++) {
            if (!isdigit((unsigned char)input[i])) {
                printf("Error: Command ID must contain only digits.\n");
                break;
            }
            
            if (i == strlen(input) - 1) {
                valid_input = true;
            }
        }
        
        if (valid_input) {
            value = (uint16_t)strtol(input, &endptr, 10);
            if (*endptr != '\0' || value > 65535) {
                printf("Error: Invalid Command ID. Must be 0-65535.\n");
                valid_input = false;
                continue;
            }
            
            /* Store as 1 word */
            eeprom_data[EEPROM_OFFSET_COMMAND_ID] = (value >> 8) & 0xFF;
            eeprom_data[EEPROM_OFFSET_COMMAND_ID + 1] = value & 0xFF;
        }
    }
    
    /* Display summary of data to be written */
    printf("\n====================================================\n");
    printf("  EEPROM Write Summary\n");
    printf("====================================================\n");
    printf("Card ID        : %c%c%c%c%c%c\n", 
           eeprom_data[EEPROM_OFFSET_CARD_ID],
           eeprom_data[EEPROM_OFFSET_CARD_ID + 1],
           eeprom_data[EEPROM_OFFSET_CARD_ID + 2],
           eeprom_data[EEPROM_OFFSET_CARD_ID + 3],
           eeprom_data[EEPROM_OFFSET_CARD_ID + 4],
           eeprom_data[EEPROM_OFFSET_CARD_ID + 5]);
    printf("Revision       : %c%c\n", 
           eeprom_data[EEPROM_OFFSET_REVISION],
           eeprom_data[EEPROM_OFFSET_REVISION + 1]);
    printf("Serial Number  : %d\n", 
           (eeprom_data[EEPROM_OFFSET_SERIAL_NUM] << 8) | 
           eeprom_data[EEPROM_OFFSET_SERIAL_NUM + 1]);
    printf("Cal. Year      : %d\n", 
           (eeprom_data[EEPROM_OFFSET_CAL_YEAR] << 8) | 
           eeprom_data[EEPROM_OFFSET_CAL_YEAR + 1]);
    printf("Cal. Month     : %d\n", 
           (eeprom_data[EEPROM_OFFSET_CAL_MONTH] << 8) | 
           eeprom_data[EEPROM_OFFSET_CAL_MONTH + 1]);
    printf("Gain Linearity : ");
    for (int i = 0; i < 8; i++) {
        printf("%02X", eeprom_data[EEPROM_OFFSET_GAIN_LINEARITY + i]);
    }
    printf("\n");
    printf("Offset Data    : ");
    for (int i = 0; i < 8; i++) {
        printf("%02X", eeprom_data[EEPROM_OFFSET_OFFSET_DATA + i]);
    }
    printf("\n");
    printf("Command ID     : %d\n", 
           (eeprom_data[EEPROM_OFFSET_COMMAND_ID] << 8) | 
           eeprom_data[EEPROM_OFFSET_COMMAND_ID + 1]);
    
    /* Confirm write */
    if (!get_confirmation("\nAre you sure you want to write this data to the EEPROM? (yes/no): ")) {
        printf("Write operation cancelled.\n");
        return;
    }
    
    /* Write data to EEPROM */
    printf("\nWriting data to EEPROM...\n");
    
    /* Seek to beginning of EEPROM */
    if (lseek(eeprom_fd, 0, SEEK_SET) < 0) {
        fprintf(stderr, "Error: Failed to seek to beginning of EEPROM: %s\n", 
                strerror(errno));
        return;
    }
    
    /* Write data */
    ssize_t bytes_written = write(eeprom_fd, eeprom_data, 32); /* Write first 32 bytes */
    if (bytes_written < 0) {
        fprintf(stderr, "Error: Failed to write to EEPROM: %s\n", 
                strerror(errno));
        return;
    }
    
    printf("Wrote %zd bytes to EEPROM\n", bytes_written);
    printf("Write operation completed successfully.\n");
    
    /* Read back for verification */
    printf("\nReading back data for verification...\n");
    read_eeprom();
}

static void read_eeprom(void)
{
    uint8_t read_buffer[EEPROM_SIZE];
    ssize_t bytes_read;
    
    printf("\n====================================================\n");
    printf("  Read EEPROM Data\n");
    printf("====================================================\n");
    
    if (current_card < 0) {
        printf("Error: No card selected. Please select a card first.\n");
        return;
    }
    
    /* Seek to beginning of EEPROM */
    if (lseek(eeprom_fd, 0, SEEK_SET) < 0) {
        fprintf(stderr, "Error: Failed to seek to beginning of EEPROM: %s\n", 
                strerror(errno));
        return;
    }
    
    /* Read data */
    bytes_read = read(eeprom_fd, read_buffer, sizeof(read_buffer));
    if (bytes_read < 0) {
        fprintf(stderr, "Error: Failed to read from EEPROM: %s\n", 
                strerror(errno));
        return;
    }
    
    printf("Read %zd bytes from EEPROM\n", bytes_read);
    
    /* Display the data */
    printf("\n====================================================\n");
    printf("  EEPROM Contents (Card %d)\n", current_card);
    printf("====================================================\n");
    printf("Card ID        : ");
    for (int i = 0; i < 6; i++) {
        printf("%02X", read_buffer[EEPROM_OFFSET_CARD_ID + i]);
    }
    printf(" (ASCII: \"");
    for (int i = 0; i < 6; i++) {
        char c = read_buffer[EEPROM_OFFSET_CARD_ID + i];
        printf("%c", isprint(c) ? c : '.');
    }
    printf("\")\n");
    
    printf("Revision       : %02X%02X\n", 
           read_buffer[EEPROM_OFFSET_REVISION], 
           read_buffer[EEPROM_OFFSET_REVISION + 1]);
    printf("Serial Number  : %d\n", 
           (read_buffer[EEPROM_OFFSET_SERIAL_NUM] << 8) | 
           read_buffer[EEPROM_OFFSET_SERIAL_NUM + 1]);
    printf("Cal. Year      : %d\n", 
           (read_buffer[EEPROM_OFFSET_CAL_YEAR] << 8) | 
           read_buffer[EEPROM_OFFSET_CAL_YEAR + 1]);
    printf("Cal. Month     : %d\n", 
           (read_buffer[EEPROM_OFFSET_CAL_MONTH] << 8) | 
           read_buffer[EEPROM_OFFSET_CAL_MONTH + 1]);
    printf("Gain Linearity : ");
    for (int i = 0; i < 8; i++) {
        printf("%02X", read_buffer[EEPROM_OFFSET_GAIN_LINEARITY + i]);
    }
    printf("\n");
    printf("Offset Data    : ");
    for (int i = 0; i < 8; i++) {
        printf("%02X", read_buffer[EEPROM_OFFSET_OFFSET_DATA + i]);
    }
    printf("\n");
    printf("Command ID     : %02X%02X\n", 
           read_buffer[EEPROM_OFFSET_COMMAND_ID], 
           read_buffer[EEPROM_OFFSET_COMMAND_ID + 1]);
    
    /* Full hex dump */
    printf("\nComplete EEPROM contents (first 64 bytes):\n");
    dump_eeprom_data(read_buffer, 64);
}

static void dump_eeprom_data(uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        if (i % 16 == 0) {
            printf("\n%04zX: ", i);
        }
        printf("%02X ", data[i]);
        if (i % 16 == 7) {
            printf(" ");
        }
    }
    printf("\n");
    
    /* ASCII representation */
    printf("\nASCII:\n");
    for (size_t i = 0; i < length; i++) {
        if (i % 16 == 0) {
            printf("\n%04zX: ", i);
        }
        printf("%c", isprint(data[i]) ? data[i] : '.');
    }
    printf("\n");
}

static bool get_confirmation(const char *prompt)
{
    char response[10];
    
    printf("%s", prompt);
    if (!fgets(response, sizeof(response), stdin)) {
        return false;
    }
    
    response[strcspn(response, "\n")] = 0;
    
    return (strcasecmp(response, "yes") == 0 || 
            strcasecmp(response, "y") == 0);
}

static void clean_stdin(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF) { }
}

static bool validate_hex_string(const char *str, size_t expected_len)
{
    if (!str || strlen(str) != expected_len) {
        return false;
    }
    
    for (size_t i = 0; i < expected_len; i++) {
        if (!isxdigit((unsigned char)str[i])) {
            return false;
        }
    }
    
    return true;
}

static bool validate_number(const char *str, int min, int max)
{
    char *endptr;
    long value;
    
    if (!str || !*str) {
        return false;
    }
    
    /* Check if it's a valid number */
    for (size_t i = 0; i < strlen(str); i++) {
        if (!isdigit((unsigned char)str[i])) {
            return false;
        }
    }
    
    /* Convert to number and validate range */
    value = strtol(str, &endptr, 10);
    if (*endptr != '\0') {
        return false;
    }
    
    return (value >= min && value <= max);
}

static bool hex_string_to_bytes(const char *hex, uint8_t *bytes, size_t len)
{
    size_t hex_len = strlen(hex);
    if (hex_len != len * 2) {
        return false;
    }
    
    for (size_t i = 0; i < len; i++) {
        char byte_str[3] = {hex[i*2], hex[i*2+1], 0};
        bytes[i] = (uint8_t)strtol(byte_str, NULL, 16);
    }
    
    return true;
}
