/* ================================================================== *\
   Pico-Remote-Analyzer.c
   St-Louys Andre - January 2023
   astlouys@gmail.com
   Revision 18-JAN-2023
   Compiler: arm-none-eabi-gcc 7.3.1
   Version 1.00

   Raspberry Pi Pico Utility to decode infrared burst sent by a remote control.
   An external monitor (or a PC running a terminal emulator program like TeraTerm)
   must be connected to the USB connector for USB CDC communication.

  
   REVISION HISTORY:
   =================
   18-JAN-2023 1.00 - Initial release.
\* ================================================================== */



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                                Include files.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                                Definitions.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
typedef unsigned int  UINT;   // processor-optimized.
typedef uint8_t       UINT8;
typedef uint16_t      UINT16;
typedef uint32_t      UINT32;
typedef uint64_t      UINT64;
typedef unsigned char UCHAR;

/* GPIO definitions. */
#define UART_TX_PIN      0              // serial line to transmit data to   an external PC running a terminal emulation software.
#define UART_RX_PIN      1              // serial line to receive  data from an external PC running a terminal emulation software.
#define IR_RX            22             // GPIO used for VS1838b infrared sensor rx.
#define PICO_LED         25             // on-board LED.
#define BUZZER           27             // active buzzer on the Geeek Pico Base.
#define ADC_VCC          29             // analog-to-digital converter of the Pico to read power supply voltage.

#define FLAG_OFF         0
#define FLAG_ON          1
#define MAX_BUTTONS      128           // maximum number of buttons on remote control.
#define MAX_IR_READINGS  500
#define TYPE_PICO        1             // microcontroller is a Pico.
#define TYPE_PICO_W      2             // microcontroller is a Pico W
#define REMOTE_FILENAME  "Samsung.c"

/* Debug flag definitions. */
#define DEBUG_NONE       0x0000000000000000
#define DEBUG_IR_COMMAND 0x0000000000000001



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                              Global variables.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
volatile UINT64 IrFinalValue[MAX_IR_READINGS];    // final timer value when receiving edge change from remote control.
volatile UINT64 IrInitialValue[MAX_IR_READINGS];  // initial timer value when receiving edge change from remote control.
volatile UCHAR  IrLevel[MAX_IR_READINGS];         // logic levels of remote control signal: 'L' (low), 'H' (high), or 'X' (undefined).
volatile UINT32 IrResultValue[MAX_IR_READINGS];   // duration of this logic level (Low or High) in the signal received from remote control.
volatile UINT16 IrStepCount;                      // number of "logic level changes" received from IR remote control in current stream.

UCHAR BrandName[128];       // brand of the remote control being analyzed.
UCHAR ButtonName[64];       // identify the remote control button being analyzed.
UCHAR LevelString[3][128];  // logic level string (high or low).
UCHAR PicoUniqueId[41];     // Pico's Unique ID.
UCHAR RemoteModel[128];     // model number of remote control being analyzed.
UCHAR Separator[256];       // horizontal bar for cosmetic purposes.

UINT8 PicoType;

UINT64 DebugBitMask;

struct
{
  UCHAR  ButtonName[64];
  UINT64 CommandId;
} RemoteData[256];
UINT16 RemoteDataTotal;



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                             Function prototypes.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* Decode last infrared burst received. */
void decode_ir_burst(UCHAR FlagAskButton);

/* Decode last infrared burst received using current remote filename. */
UINT8 decode_ir_command(UINT8 *IrCommand);

/* Display the infrared burst timing information. */
void display_burst_timing(UINT8 FlagAskButton);

/* Display complete list of buttons decoded. */
void display_button_list(void);

/* Display header for burst timing information. */
void display_header(void);

/* Assign brand name and serial number to the remote control. */
void enter_remote_id(void);

/* Determine if the microcontroller is a Pico or a Pico W and retrieve its Unique Number. */
UINT8 get_pico_id(void);

/* Initialize variables that will receive next infrared data burst. */
void init_burst_variables(void);

/* Read a string from stdin. */
void input_string(UCHAR *String);

/* Interrupt handler for signal received from IR sensor. */
gpio_irq_callback_t isr_signal_trap(UINT8 gpio, UINT32 Events);

/* Make a tone for the specified number of milliseconds on active buzzer. */
void tone(UINT16 MilliSeconds);

/* Send a string to external monitor through Pico UART (or USB CDC). */
void uart_send(UINT16 LineNumber, UCHAR *String);



/* $PAGE */
/* $TITLE=main() */
/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                           Main program entry point.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
int main(void)
{
  UCHAR String[256];

  UINT8 FlagAskButton;
  UINT8 IrCommand;

  UINT Loop1UInt;
  UINT Menu;


  /* Debug items. */
  DebugBitMask  = DEBUG_NONE;
  // DebugBitMask += DEBUG_IR_COMMAND;


  /* Initializations. */
  IrStepCount     = 0;  // number of "logic level changes" in the infrared burst.
  RemoteDataTotal = 0;  // number of buttons already decoded on remote unit.
  strcpy(LevelString[0], "low");
  strcpy(LevelString[1], "high");
  strcpy(LevelString[2], "---");
  strcpy(Separator, "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =\r");
  strcpy(RemoteModel, "TBD");
  strcpy(BrandName, REMOTE_FILENAME);
  for (Loop1UInt = 0; Loop1UInt < strlen(BrandName); ++Loop1UInt)
    if (BrandName[Loop1UInt] == '.') BrandName[Loop1UInt] = 0x00;  // force end-of-string before ".c" file extension

  for (Loop1UInt = 0; Loop1UInt < MAX_BUTTONS; ++Loop1UInt)
  {
    RemoteData[Loop1UInt].ButtonName[0] = 0x00;  // null string.
    RemoteData[Loop1UInt].CommandId     = 0ll;   // invalid command.
  }


  /* Initialize UART0 used to send information to a PC terminal emulator program through USB CDC. */
  stdio_init_all();

  /* Initialize UART0 used for bi-directional communication with a PC running TeraTermn (or other) terminal emulation software. */
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_init(uart0, 921600);
  uart_set_format(uart0, 8, 1, UART_PARITY_NONE);

  /* Initialize Pico's analog-to-digital (ADC) converter used to determine if we are running on a Pico or a Pico W. */
  adc_init();
  adc_gpio_init(ADC_VCC);    // power supply voltage.

  /* GPIO used for active buzzer. */
  gpio_init(BUZZER);
  gpio_set_dir(BUZZER, GPIO_OUT);

  /* Initialize VS1838b (infrared sensor) GPIO. It will receive infrared bursts from remote control. */
  gpio_init(IR_RX);
  gpio_set_dir(IR_RX, GPIO_IN);
  gpio_pull_up(IR_RX);  // Line will remain at high level until a signal is received.


  /* Determine microcontroller type (Pico or Pico W) and Pico's Unique ID ("serial number"). */
  get_pico_id();


  /* Wait for a valid connection with the PC terminal emulator software (USB CDC). */
  while (!stdio_usb_connected())
  {
    tone(25);
    sleep_ms(2500);
  }
  

  /* Initialize the interrupt service routine (ISR) used to time-tag the infrared burst. */
  gpio_set_irq_enabled_with_callback(IR_RX, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&isr_signal_trap);


  /* Confirm / enter remote control brand and model number on entry. */
  enter_remote_id();


  /* $PAGE */
  /* ------------------------------------------------------------------ *\
                    Main system loop (will loop forever).
  \* ------------------------------------------------------------------ */
  while (1)
  {
    /* Initialize variables that will receive next infrared data burst. */
    init_burst_variables();


    printf("\r\r");
    printf("Current step count is: %u\r\r\r", IrStepCount);
    printf("Press a button on remote control for analysis: ");
    
    /* Wait until a button has been pressed on remote control. */
    while (IrStepCount == 0) sleep_ms(250);
    printf("\r\r\r");
    sleep_ms(250);  // make sure infrared data burst has been completed.

    display_header();


    printf("Current step count is: %u\r\r\r", IrStepCount);



    printf("     1) Assign remote control brand name and model number.\r");
    printf("     2) Display infrared burst timing.\r");
    printf("     3) Decode this infrared burst using file %s\r", REMOTE_FILENAME);
    printf("     4) Display complete remote control button list.\r");
    printf("\r");

    printf("        Enter an option: ");
    input_string(String);
    if (String[0] == 0x0D) continue;
    Menu = atoi(String);
    
    
    switch(Menu)
    {
      case (1):
        /* Assign remote control brand and model number. */
        printf("\r\r");
        enter_remote_id();
        printf("\r\r");
      break;

      case (2):
        /* Display infrared burst timing info. */
        printf("\r\r");
        display_burst_timing(FLAG_ON);
        printf("\r\r");
      break;
      
      case (3):
        /* Decode last infrared burst received. */
        printf("\r\r");
        decode_ir_burst(FLAG_ON);
        printf("\r\r");
      break;

      case (4):
        /* Display complete button list. */
        printf("\r\r");
        display_button_list();
        printf("\r\r");
      break;

      default:
        printf("\r\r");
        printf("           Invalid choice... please re-enter [%s]  [%u]\r\r\r\r\r", String, Menu);
        printf("\r\r");
      break;
    }
  }
}





/* $PAGE */
/* $TITLE=decode_ir_command() */
/* ------------------------------------------------------------------ *\
                 Decode last infrared burst received.
\* ------------------------------------------------------------------ */
#include REMOTE_FILENAME





/* $PAGE */
/* $TITLE=decode_ir_burst() */
/* ------------------------------------------------------------------ *\
                 Decode last infrared burst received.
\* ------------------------------------------------------------------ */
void decode_ir_burst(UINT8 FlagAskButton)
{
  UCHAR String[256];

  UINT8 IrCommand;
  UINT8 LineCount;

  UINT16 Loop1UInt16;

  
  /* Initializations. */
  LineCount = 50;  // number of lines per page.


  if (IrStepCount == 0)
  {
    printf("No infrared burst has been received yet...\r");
    printf("You must first press a button on the remote control before selecting this menu choice.\r\r");
    printf("Press <Enter> to return to menu: ");

    input_string(String);

    return;
  }


  if (FlagAskButton)
  {
    printf("Enter button name for this infrared burst: ");
    input_string(ButtonName);
  }
  

  display_burst_timing(FLAG_OFF);  // first, display infrared burst timing.
  decode_ir_command(&IrCommand);   // then, display decoded data.

  /*** Optionally display buttons already decoded so far. ***
  display_header();

  printf("\r");
  printf("Number of buttons decoded: %u\r\r", RemoteDataTotal);
  printf("        Remote control             Infrared command\r");
  printf("           button name                      decoded\r\r");

  for (Loop1UInt16 = 0; Loop1UInt16 < MAX_BUTTONS; ++Loop1UInt16)
  {
    printf("[%3u] %16s                   0x%8.8llX\r", Loop1UInt16, RemoteData[Loop1UInt16].ButtonName, RemoteData[Loop1UInt16].CommandId);
    if (((Loop1UInt16 % LineCount) == 0) && (Loop1UInt16 != 0))
    {
      printf("\r");
      printf("to be continued...\r");
      printf("%s\r\r\r", Separator);
      display_header();
      printf("\r");
      printf("Number of buttons decoded: %u\r\r", RemoteDataTotal);
      printf("        Remote control             Infrared command\r");
      printf("           button name                      decoded\r\r");
    }
  }
  printf("%s\r\r\r", Separator);
  ***/

  return;
}





/* $PAGE */
/* $TITLE=display_burst_timing() */
/* ------------------------------------------------------------------ *\
            Display the infrared burst timing information.
\* ------------------------------------------------------------------ */
void display_burst_timing(UINT8 FlagAskButton)
{
  UCHAR Dum1UChar[128];
  UCHAR String[128];

  UINT16 CurrentHiCount;
  UINT16 Loop1UInt16;
  UINT16 Loop2UInt16;
  UINT16 LineCount;


  /* Initializations. */
  LineCount = 50;  // number of lines per page.


  if (IrStepCount == 0)
  {
    printf("No infrared burst has been received yet...\r");
    printf("You must first press a button on the remote control before selecting this menu choice.\r\r");
    printf("Press <Enter> to return to menu: ");

    input_string(String);

    return;
  }


  if (FlagAskButton)
  {
    printf("Enter button name for this infrared burst: ");
    input_string(ButtonName);
  }
  

  for (Loop1UInt16 = 0; Loop1UInt16 < IrStepCount; Loop1UInt16 += (LineCount * 2))
  {
    printf("\r\r\r\r\r");
    display_header();

    /* Display Button name. */
    printf("Button: %s\r\r", ButtonName);

    /* Display timing for every logic level change of last received infrared burst. */
    printf(" Step      Logic    Duration                     Step      Logic    Duration\r");
    printf("number     level                                number     level\r\r");

    for (Loop2UInt16 = Loop1UInt16; Loop2UInt16 < (Loop1UInt16 + LineCount); ++Loop2UInt16)
    {
      printf("  %3u       %4s      %5lu", Loop2UInt16 + 1, LevelString[IrLevel[Loop2UInt16]], IrResultValue[Loop2UInt16]);

      if (IrStepCount > (Loop2UInt16 + LineCount))
      {
        printf("                       %3u       %4s      %5lu", Loop2UInt16 + LineCount + 1, LevelString[IrLevel[Loop2UInt16 + LineCount]], IrResultValue[Loop2UInt16 + LineCount]);
      }
      printf("\r");

      if (Loop2UInt16 == IrStepCount)
      {
        printf("\r");
        printf("%s", Separator);
        break;  // get out of "for" loop when all steps have been displayed.
      }
    }

    if (IrStepCount > (Loop1UInt16 + (LineCount * 2)))
    {
      printf("\r");
      printf("to be continued\r");
    }
    printf("%s", Separator);
  }

  return;
}
 




/* $PAGE */
/* $TITLE=display_button_list() */
/* ------------------------------------------------------------------ *\
               Display complete list of buttons decoded.
\* ------------------------------------------------------------------ */
void display_button_list(void)
{
  UINT8  LineCount;

  UINT16 Loop1UInt16;


  /* Initializations. */
  LineCount = 50;


  display_header();
  printf("\r");
  printf("Number of buttons decoded: %u\r\r", RemoteDataTotal);
  printf("        Remote control             Infrared command\r");
  printf("           button name                      decoded\r\r");

  for (Loop1UInt16 = 0; Loop1UInt16 < RemoteDataTotal; ++Loop1UInt16)
  {
    printf("[%3u] %16s                   0x%8.8llX\r", Loop1UInt16, RemoteData[Loop1UInt16].ButtonName, RemoteData[Loop1UInt16].CommandId);
    
    if (((Loop1UInt16 % LineCount) == 0) && (Loop1UInt16 != 0))
    {
      printf("\r");
      printf("to be continued...\r");
      printf("%s\r\r\r", Separator);
      display_header();
      printf("\r");
      printf("Number of buttons decoded: %u\r\r", RemoteDataTotal);
      printf("        Remote control             Infrared command\r");
      printf("           button name                      decoded\r\r");
    }
  }
  printf("%s\r\r\r", Separator);

  return;
}





/* $PAGE */
/* $TITLE=display_header() */
/* ------------------------------------------------------------------ *\
              Display header for burst timing information.
\* ------------------------------------------------------------------ */
void display_header(void)
{
  UCHAR String[128];

  UINT16 Loop1UInt16;


  printf("%s", Separator);
  

  sprintf(String, "Flash-Remote-Analyzer\r");
  for (Loop1UInt16 = 0; Loop1UInt16 < ((strlen(Separator) - strlen(String)) / 2); ++Loop1UInt16)
    printf(" ");
  printf("%s", String);
  

  sprintf(String, "Microcontroller is a ");
  if (PicoType == TYPE_PICO)
    sprintf(&String[strlen(String)], "Pico\r");
  else
    sprintf(&String[strlen(String)], "Pico W\r");
  for (Loop1UInt16 = 0; Loop1UInt16 < ((strlen(Separator) - strlen(String)) / 2); ++Loop1UInt16)
    printf(" ");
  printf("%s", String);
  

  sprintf(String, "Pico's Unique ID: %s\r", PicoUniqueId);
  for (Loop1UInt16 = 0; Loop1UInt16 < ((strlen(Separator) - strlen(String)) / 2); ++Loop1UInt16)
    printf(" ");
  printf("%s", String);
  

  sprintf(String, "Brand under analysis: %s\r", BrandName);
  for (Loop1UInt16 = 0; Loop1UInt16 < ((strlen(Separator) - strlen(String)) / 2); ++Loop1UInt16)
    printf(" ");
  printf("%s", String);


  sprintf(String, "Remote control model number: %s\r", RemoteModel);
  for (Loop1UInt16 = 0; Loop1UInt16 < ((strlen(Separator) - strlen(String)) / 2); ++Loop1UInt16)
    printf(" ");
  printf("%s", String);


  sprintf(String, "Step count: %u\r", IrStepCount);
  for (Loop1UInt16 = 0; Loop1UInt16 < ((strlen(Separator) - strlen(String)) / 2); ++Loop1UInt16)
    printf(" ");
  printf("%s", String);


  printf("%s", Separator);
}





/* $PAGE */
/* $TITLE=enter_remote_id() */
/* ------------------------------------------------------------------ *\
        Assign brand name and serial number to the remote control.
\* ------------------------------------------------------------------ */
void enter_remote_id(void)
{
  UCHAR Dum1UChar[64];


  printf("Current remote control brand is %s\r", BrandName);
  printf("Enter the brand if it must be different: ");
  input_string(Dum1UChar);
  if ((Dum1UChar[0] != 0x00) && (Dum1UChar[0] != 0x0D))
    strcpy(BrandName, Dum1UChar);
  printf("\r\r");


  printf("Current remote control model number is %s\r", RemoteModel);
  printf("Enter the remote model number if it must be different: ");
  input_string(Dum1UChar);
  if ((Dum1UChar[0] != 0x00) && (Dum1UChar[0] != 0x0D))
    strcpy(RemoteModel, Dum1UChar);
  printf("\r\r");

  return;
}





/* $PAGE */
/* $TITLE=get_pico_id() */
/* ------------------------------------------------------------------ *\
         Determine if the microcontroller is a Pico or a Pico W
                    and retrieve its Unique Number.
\* ------------------------------------------------------------------ */
UINT8 get_pico_id(void)
{
  UINT8 Loop1UInt8;
  
  UINT16 AdcValue1;
  UINT16 AdcValue2;

  float Volts1;
  float Volts2;

  pico_unique_board_id_t board_id;  // Pico's Unique ID.


  /* Select power supply input. */
  adc_select_input(3);
  gpio_put(PICO_LED, true);

  /* Read ADC converter raw value. */
  AdcValue1 = adc_read();

  /* Convert raw value to voltage value. */
  Volts1 = (3 * (AdcValue1 * 3.3f / 4096));



  /* The important power supply value to consider is when GPIO25 is Low. */
  gpio_put(PICO_LED, false);
    
  /* Read ADC converter raw value. */
  AdcValue2 = adc_read();

  /* Convert raw value to voltage value. */
  Volts2 = (3 * (AdcValue2 * 3.3f / 4096));
  if (Volts2 > 3)
    PicoType = TYPE_PICO;
  else
    PicoType = TYPE_PICO_W;


  /* Retrieve Pico Unique Number from flash memory IC. */
  pico_get_unique_board_id(&board_id);


  /* Build-up the Unique ID string in hex. */
  PicoUniqueId[0] = 0x00;  // initialize as null string.
  for (Loop1UInt8 = 0; Loop1UInt8 < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++Loop1UInt8)
  {
    sprintf(&PicoUniqueId[strlen(PicoUniqueId)], "%2.2X", board_id.id[Loop1UInt8]);
    if ((Loop1UInt8 % 2) && (Loop1UInt8 != 7))
      sprintf(&PicoUniqueId[strlen(PicoUniqueId)], "-");
  }
  
  return PicoType;
}





/* $PAGE */
/* $TITLE=init_burst_variables() */
/* ------------------------------------------------------------------ *\
    Initialize variables that will receive next infrared data burst.
\* ------------------------------------------------------------------ */
void init_burst_variables(void)
{
  UINT Loop1UInt;


  IrStepCount = 0;

  for (Loop1UInt = 0; Loop1UInt < MAX_IR_READINGS; ++Loop1UInt)
  {
    IrInitialValue[Loop1UInt] = 0ll;
    IrFinalValue[Loop1UInt]   = 0ll;
    IrResultValue[Loop1UInt]  = 0ll;
    IrLevel[Loop1UInt]        = 2;
  }
}





/* $PAGE */
/* $TITLE=input_string() */
/* ------------------------------------------------------------------ *\
                       Read a string from stdin.
\* ------------------------------------------------------------------ */
void input_string(UCHAR *String)
{
  int8_t DataInput;

  UINT8 Loop1UInt8;


  Loop1UInt8 = 0;
  do
  {
    DataInput = getchar_timeout_us(50000);

    switch (DataInput)
    {
      case (PICO_ERROR_TIMEOUT):
      case (0):
        continue;
      break;

      case (8):
        /* <Backspace> */
        if (Loop1UInt8 > 0)
        {
          --Loop1UInt8;
          String[Loop1UInt8] = 0x00;
          printf("%c %c", 0x08, 0x08);  // erase character under the cursor.
        }
      break;

      case (0x0D):
        /* <Enter> */
        if (Loop1UInt8 == 0)
        {
          String[Loop1UInt8++] = (UCHAR)DataInput;  
          String[Loop1UInt8++] = 0x00;
        }
        printf("\r");
      break;

      default:
        printf("%c", (UCHAR)DataInput);
        String[Loop1UInt8] = (UCHAR)DataInput;
        // printf("Loop1UInt8: %3u   %2.2X - %c\r", Loop1UInt8, DataInput, DataInput);  ///
        ++Loop1UInt8;
      break;
    }
  } while((Loop1UInt8 < 128) && (DataInput != 0x0D));
  
  String[Loop1UInt8] = '\0';  // end-of-string

  /***
  for (Loop1UInt8 = 0; Loop1UInt8 < 10; ++Loop1UInt8)
    printf("%2u:[%2.2X]   ", Loop1UInt8, String[Loop1UInt8]);
  printf("\r");
  ***/

  return;
}





/* $PAGE */
/* $TITLE=isr_signal_trap() */
/* ----------------------------------------------------------------- *\
         Interrupt handler for signal received from IR sensor
                           (type VS1838B).
\* ----------------------------------------------------------------- */
gpio_irq_callback_t isr_signal_trap(UINT8 gpio, UINT32 Events)
{
  if (gpio == IR_RX)
  {
    /* IR line goes from Low to High. */
    if (Events & GPIO_IRQ_EDGE_RISE)
    {
      IrFinalValue[IrStepCount]   = time_us_64();                                                       // this is the final timer value for current Low level.
      IrResultValue[IrStepCount]  = (UINT32)(IrFinalValue[IrStepCount] - IrInitialValue[IrStepCount]);  // calculate duration of current Low level.
      IrLevel[IrStepCount]        = 0;                                                                  // identify  as Low level.
      ++IrStepCount;                                                                                    // start next logic level change.
      IrInitialValue[IrStepCount] = time_us_64();                                                       // this is also start timer of next High level.

      gpio_acknowledge_irq(IR_RX, GPIO_IRQ_EDGE_RISE);
   }


    /* IR line goes from High to Low. */
    if (Events & GPIO_IRQ_EDGE_FALL)
    {
      if (IrStepCount > 0)
      {
        IrFinalValue[IrStepCount]  =  time_us_64();                                                      // this is the final timer value for current High level.
        IrResultValue[IrStepCount] = (UINT32)(IrFinalValue[IrStepCount] - IrInitialValue[IrStepCount]);  // calculate duration of current High level.
        IrLevel[IrStepCount]       = 1;                                                                  // identify as High level.
        ++IrStepCount;                                                                                   // start next logic level change.
      }
      IrInitialValue[IrStepCount]  = time_us_64();                                                       // this is also start timer of next Low level.
      
      gpio_acknowledge_irq(IR_RX, GPIO_IRQ_EDGE_FALL);
    }
  }
}





/* $PAGE */
/* $TITLE=tone() */
/* ------------------------------------------------------------------ *\
         Make a tone for the specified number of milliseconds on
                             active buzzer.
\* ------------------------------------------------------------------ */
void tone(UINT16 MilliSeconds)
{
  gpio_put(BUZZER, 1);
  sleep_ms(MilliSeconds);
  gpio_put(BUZZER, 0);

  return;
}





/* $PAGE */
/* $TITLE=uart_send() */
/* ------------------------------------------------------------------ *\
    Send a string to external monitor through Pico UART (or USB CDC).
\* ------------------------------------------------------------------ */
void uart_send(UINT16 LineNumber, UCHAR *String)
{
  UCHAR LineString[512];

  
  /* Initializations. */
  LineString[0] = 0x00;  // end-of-string.


  /* Trap special control code for <home>. Replace "home" by appropriate escape sequence for "home" on a VT101. */
  if (strcmp(String, "home") == 0)
  {
    String[0] = 0x1B; // ESC code
    String[1] = '[';
    String[2] = 'H';
    String[3] = 0x00;
  }

  /* Trap special control code for <cls>. Replace "cls" by appropriate escape sequence for "clear screen" on a VT101. */
  if (strcmp(String, "cls") == 0)
  {
    /* For some reason, "cls" has side effects on the behaviour of uart_send() for future logging and should be avoided. */
    String[0] = 0x1B;
    String[1] = '[';
    String[2] = '2';
    String[3] = 'J';
    String[4] = 0x00;
  }

  /* "Timer stamp" will not be printed if first character is '-',
     or if first character is a line feed '\r' when we simply want to add line spacing in the debug log,
     or if the string represents a tag for an an escape sequence (for example 'Home' or "Clear screen'). */
  if ((String[0] != '-') && (String[0] != '\r') && (String[0] != 0x1B) && (String[0] != '|'))
  {
    /* Send line number through UART. */
    sprintf(LineString, "[%7lu] ", LineNumber);

    /* Send current timer value to external terminal. */
    sprintf(&LineString[strlen(LineString)], "[%10lu] ", time_us_32());
  }

  /* Send log string through UART. */
  sprintf(&LineString[strlen(LineString)], String);
  printf(LineString);

  return;
}
