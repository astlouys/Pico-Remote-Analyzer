/* $TITLE=decode_ir_command() */
/* $PAGE */
/* ------------------------------------------------------------------ *\
              Function to decode an infrared command
       received using Samsung BN59-00673A remote control unit.
 
                  Samsung protocol white paper:
                      Carrier: (37.9 kHz)
         1 Start bit (or "get-ready" or "wake-up" bit made of:
      4450 micro-seconds Low level / 4450 micro-seconds High level

      32 Data bits made of:
             bit 0 = 550 micro-seconds Low /   550 micro-seconds High
             bit 1 = 550 micro-seconds Low /  1675 micro-seconds High

    NOTE: All "brand-related" parameters must be kept in this function.
          This allows to replace only this include file with another
          "remote control brand" include file to support more remote
          controls while keeping the rest of the Firmware untouched.
\* ------------------------------------------------------------------ */
#include "Samsung.h"

#define NUMBER_OF_BITS             32  // number of bits in the infrared data stream.
#define NUMBER_OF_STEPS           135  // normal count for total number of steps for this remote control unit.
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
    case (0xE0E040BF):
      /* Button "Power". */
      printf("<Power>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0D827):
      /* Button "TV". */
      printf("<TV>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E020DF):
      /* Button "1". */
      printf("<1>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0A05F):
      /* Button "2". */
      printf("<2>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0609F):
      /* Button "3". */
      printf("<3>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E010EF):
      /* Button "4". */
      printf("<4>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0906F):
      /* Button "5". */
      printf("<5>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E050AF):
      /* Button "6". */
      printf("<6>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E030CF):
      /* Button "7". */
      printf("<7>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0B04F):
      /* Button "8". */
      printf("<8>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0708F):
      /* Button "9". */
      printf("<9>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
   break;

    case (0xE0E08877):
      /* Button "0". */
      printf("<0>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0C43B):
      /* Button "-". */
      printf("<->\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0C837):
      /* Button "Pre-Ch". */
      printf("<Pre-Ch>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0F00F):
      /* Button "Mute". */
      printf("<Mute>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0807F):
      /* Button "Source". */
      printf("<Source>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0E01F):
      /* Button "Volume Up". */
      printf("<Volume Up>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
     break;

    case (0xE0E0D02F):
      /* Button "Volume Down". */
      printf("<Volume Down>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
   break;

    case (0xE0E048B7):
      /* Button "Channel Up". */
      printf("<Channel Up>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E008F7):
      /* Button "Channel Down". */
      printf("<Channel Down>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E058A7):
      /* Button "Menu". */
      printf("<Menu>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0D629):
      /* Button "Ch List". */
      printf("<Ch List\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E031CE):
      /* Button "W. Link". */
      printf("<W. Link>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0D22D):
      /* Button "Tools". */
      printf("<Tools>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E01AE5):
      /* Button "Return". */
      printf("<Return>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0F807):
      /* Button "Info". */
      printf("<Info>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  /// assign the command to be executed here.
    break;

    case (0xE0E0B44B):
      /* Button "Exit". */
      printf("<Exit>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E006F9):
      /* Button "Up". */
      printf("<Up>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E08679):
      /* Button "Down". */
      printf("<Down>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E0A659):
      /* Button "Left". */
      printf("<Left>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E046B9):
      /* Button "Right". */
      printf("<Right>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E016E9):
      /* Button "Enter". */
      printf("<Enter>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E036C9):
      /* Button "Red". */
      printf("<Red>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E028D7):
      /* Button "Green". */
      printf("<Green>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E0A857):
      /* Button "Yellow". */
      printf("<Yellow>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E06897):
      /* Button "Blue". */
      printf("<Blue>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E0A45B):
      /* Button "CC". */
      printf("<CC>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E000FF):
      /* Button "MTS". */
      printf("<MTS>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E0C639):
      /* Button "DMA". */
      printf("<DMA>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E029D6):
      /* Button "E.Mode". */
      printf("<E.Mode>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E07C83):
      /* Button "P.Size". */
      printf("<P.Size>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E022DD):
      /* Button "Fav.Ch.". */
      printf("<Fav.Ch.>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E0A25D):
      /* Button "Rewind". */
      printf("<Rewind>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E052AD):
      /* Button "Pause". */
      printf("<Pause>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E012ED):
      /* Button "Forward". */
      printf("<Forward>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E0E21D):
      /* Button "Play". */
      printf("<Play>\r\r");
      *IrCommand = IR_COMMAND_TO_EXECUTE;  // assign the command to be executed here.
    break;

    case (0xE0E0629D):
      /* Button "Stop". */
      printf("<Stop>\r\r");
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
