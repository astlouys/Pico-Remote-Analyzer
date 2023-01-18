/* $TITLE=decode_ir_command() */
/* $PAGE */
/* ------------------------------------------------------------------ *\
              Function to decode an infrared command
       received using Memorex MCR 5221 remote control unit.
 
                  Memorex protocol white paper:
                      Carrier: (37.9 kHz)
         1 Start bit (or "get-ready" or "wake-up" bit made of:
      4450 micro-seconds Low level / 4450 micro-seconds High level

      32 Data bits made of:
             bit 0 = 475 micro-seconds Low /   650 micro-seconds High
             bit 1 = 475 micro-seconds Low /  1750 micro-seconds High

    NOTE: All "brand-related" parameters must be kept in this function.
          This allows to replace only this include file with another
          "remote control brand" include file to support more remote
          controls while keeping the rest of the Firmware untouched.
\* ------------------------------------------------------------------ */
#include "Memorex.h"

#define NUMBER_OF_BITS             32  // number of bits in the infrared data stream.
#define NUMBER_OF_STEPS            73  // normal count for total number of steps for this remote control unit.
#define NUMBER_OF_WAKEUP_STEPS      2  // number of steps in the "get-ready" / "start bit" / "wake-up".
#define SEPARATOR               10000  // a duration greater than 10000 usec is considered a separator.
#define TRIGGER_POINT_0_1         750  // trigger point between a "0" bit and a "1" bit (shorter than 750 usec = 0  /  longer that 750 usec = 1)


UINT8 decode_ir_command(UINT8 *IrCommand)
{
  UCHAR Dum1Str[64];
  UCHAR String[256];

  UINT8 BitNumber;
  UINT8 FlagError;          // indicate an error in remote control packet received.
  
  UINT16 Loop1UInt16;
  UINT16 Loop2UInt16;

  UINT64 DataBuffer;


  /* Initialization. */
  BitNumber  = 0;
  DataBuffer = 0ll;       // data stream received from IR remote control.
  *IrCommand = 0;         // initialize as zero on entry.
  FlagError  = FLAG_OFF;  // assume no error on entry.


  printf("\r");
  printf("Decoding infrared burst with algorithm: %s\r\r", REMOTE_FILENAME);
  printf("Total number of steps / logic level changes: %3u (should be %3u)\r", IrStepCount, NUMBER_OF_STEPS);
  printf("\r");
 
 
  display_header();
  
  /* Display Button name. */
  printf("Button: %s\r\r", ButtonName);



  /* Analyze and process each step change in remote control burst. */
  /* Display header. */
  printf("Event       Bit       Level   Duration        Level   Duration      Result\r");
  printf("number     number\r\r");


  for (Loop1UInt16 = 0; Loop1UInt16 < IrStepCount; Loop1UInt16 += 2)
  {
    if (Loop1UInt16 >= NUMBER_OF_WAKEUP_STEPS)
      BitNumber = (((Loop1UInt16 - NUMBER_OF_WAKEUP_STEPS) / 2) + 1);
    else
    {
      /* Display two <Get ready> steps from IR burst. */
      printf("[%3u]       ---       %4s      %5lu         %4s      %5lu     <get ready>\r", Loop1UInt16, LevelString[IrLevel[Loop1UInt16]], IrResultValue[Loop1UInt16], LevelString[IrLevel[Loop1UInt16 + 1]], IrResultValue[Loop1UInt16 + 1]);
      continue;
    }
          
          
    if ((BitNumber > 0) && (BitNumber <= NUMBER_OF_BITS))
    {
      DataBuffer <<= 1;
      if (IrResultValue[Loop1UInt16 + 1] > TRIGGER_POINT_0_1) ++DataBuffer;  // High level determines if this is a 0 or 1.
  
      /* Display data bits. */
      printf("[%3u]       %3u       %4s      %5lu         %4s      %5lu      0x%8.8llX\r", Loop1UInt16, BitNumber, LevelString[IrLevel[Loop1UInt16]], IrResultValue[Loop1UInt16], LevelString[IrLevel[Loop1UInt16 + 1]], IrResultValue[Loop1UInt16 + 1], DataBuffer);
    }


    if (BitNumber > NUMBER_OF_BITS)
    {
      /* Display extra bits. For this remote, it is a copy of the first 32 bits. */
      printf("[%3u]       ---       %4s      %5lu         %4s      %5lu\r", Loop1UInt16, LevelString[IrLevel[Loop1UInt16]], IrResultValue[Loop1UInt16], LevelString[IrLevel[Loop1UInt16 + 1]], IrResultValue[Loop1UInt16 + 1]);
    }
  
  
    /* When reading a value that makes no sense, assume that we passed the last valid value of the IR stream. */
    if ((IrResultValue[Loop1UInt16] > SEPARATOR) || (IrResultValue[Loop1UInt16 + 1] > SEPARATOR))
    {
        printf("---------------------------- Reaching end of data bits at Step %4u\r", Loop1UInt16);
    }
    else
    {
      for (Loop2UInt16 - 0; Loop2UInt16 < 2; ++Loop2UInt16)
      {
        if (IrLevel[Loop1UInt16 + Loop2UInt16] == 0)
        {
          /* Low level, it is a first half bit. Make a rough validation only. */
          if (IrResultValue[Loop1UInt16 + Loop2UInt16] > TRIGGER_POINT_0_1)
          {
            FlagError = FLAG_ON;

            sprintf(String, "decode_ir_command() - Error IrLevel <L>   Event number: %u   IrResultValue: %lu\r", Loop1UInt16 + Loop2UInt16, IrResultValue[Loop1UInt16 + Loop2UInt16]);
            uart_send(__LINE__, String);
          }
        }
        else
        {
          /* High level, it is a second half bit... assume it is a "zero" bit on entry. */
          DataBuffer <<= 1;

          /* Now check if our assumption was correct. */
          if ((IrResultValue[Loop1UInt16 + Loop2UInt16] > TRIGGER_POINT_0_1) && (IrResultValue[Loop1UInt16 + Loop2UInt16] < (TRIGGER_POINT_0_1 * 4)))
          {
            /* It was a "one" bit. DataBuffer has already been shifted left above, simply add 1 for current "one" bit. */
            ++DataBuffer;
          }
        }
      }
    }
  }

  printf("%s\r", Separator);
  printf("Final data: 0x%8.8llX     Final step count: %2u (should be %u)\r\r", DataBuffer, IrStepCount, NUMBER_OF_STEPS);
  printf("%s\r\r", Separator);


  printf("Press <x> to record this button...\r");
  printf("or <Enter> to return to menu: ");
  input_string(Dum1Str);
  if ((Dum1Str[0] == 'x') || (Dum1Str[0] == 'X'))
  {
    strcpy(RemoteData[RemoteDataTotal].ButtonName, ButtonName);
    RemoteData[RemoteDataTotal].CommandId = DataBuffer;
    ++RemoteDataTotal;
  }

  
  
  return IR_COMMAND_TO_EXECUTE;  /// let the program fall below when adding infrared decoding to a project.



  switch (DataBuffer)
  {
    case (0x2525609F):
      /* Button "Power". */
      printf("<Power>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x25257887):
      /* Button "CD door". */
      printf("<CD Door>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525807F):
      /* Button "1". */
      printf("<1>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525906F):
      /* Button "2". */
      printf("<2>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x25258877):
      /* Button "3". */
      printf("<3>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x25259867):
      /* Button "4". */
      printf("<4>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252540BF):
      /* Button "5". */
      printf("<5>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252550AF):
      /* Button "6". */
      printf("<6>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252548B7):
      /* Button "7". */
      printf("<7>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252558A7):
      /* Button "8". */
      printf("<8>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525C03F):
      /* Button "9". */
      printf("<9>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
   break;

    case (0x2525D02F):
      /* Button "0". */
      printf("<0>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525C837):
      /* Button "Over". */
      printf("<Over>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252505FA):
      /* Button "Mute". */
      printf("<Mute>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252530CF):
      /* Button "Stop". */
      printf("<Stop>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252520DF):
      /* Button "Play / Pause". */
      printf("<Play / Pause>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525B04F):
      /* Button "Rewind / Down". */
      printf("<Rewind / Down>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
     break;

    case (0x2525A05F):
      /* Button "Fast forward / Up". */
      printf("<Fast Forward / Up>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
   break;

    case (0x252504FB):
      /* Button "Volume up". */
      printf("<Volume Up>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252506F9):
      /* Button "Volume down". */
      printf("<Volumn Down>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252538C7):
      /* Button "Random / Down". */
      printf("<Random / Down>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525D827):
      /* Button "Repeat / Up". */
      printf("<Repeat / Up>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x252528D7):
      /* Button "Set / Memory / Clock". */
      printf("<Set / Memory / Clock>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525708F):
      /* Button "Tuner". */
      printf("<Tuner>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x25256897):
      /* Button "CD". */
      printf("<CD>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525B847):
      /* Button "Time". */
      printf("<Time>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0x2525A857):
      /* Button "Display". */
      printf("<Display>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    default:
      /* Unrecognized. */
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
      sprintf(String, "Unrecognized IR command: 0x%8.8X\r", DataBuffer);
      uart_send(__LINE__, String);
      FlagError = FLAG_ON;
    break;
  }



  if (FlagError == FLAG_ON)
  {
    return 1;
  }
  else
  {
    tone(50);  // audible feedback when valid IR command has been decoded.
    return 0;
  }
}
