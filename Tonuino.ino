/*-----------------------------------------------------------------------------------------
              MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
              Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
  Signal      Pin          Pin           Pin       Pin        Pin              Pin
  -----------------------------------------------------------------------------------------
  RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
  SPI SS      SDA(SS)      10            53        D10        10               10
  SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
  SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
  SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15

  button assignments:
  -------------------

  Button B0 (by default pin A0, middle button on the original TonUINO): play+pause
  Button B1 (by default pin A1, right button on the original TonUINO): volume up
  Button B2 (by default pin A2, left button on the original TonUINO): volume down

  additional button actions:
  --------------------------

  Hold B0 for 5 seconds during idle: Enter erase nfc tag mode
  Hold B0 for 5 seconds during playback in story book mode: Reset progress to track 1
  Hold B0 for 2 seconds during erase nfc tag mode: Cancel erase nfc tag mode
  Hold B0 for 2 seconds during nfc tag setup mode: Cancel nfc tag setup mode
  Click B0 during nfc tag setup mode: Confirm selection
  Double click B0 during nfc tag setup mode: Announce folder number or track number

  Hold B1 for 2 seconds during playback in album, party and story book mode: Skip to the next track
  Hold B1 for 2 seconds during nfc tag setup mode: Jump 10 folders or 10 tracks forward

  Hold B2 for 2 seconds during playback in album, party and story book mode: Skip to the previous track
  Hold B2 for 2 seconds during nfc tag setup mode: Jump 10 folders or 10 tracks backwards

  Hold B0 + B1 + B2 while powering up: Erase the eeprom contents. (Use with care, eeprom write/erase cycles are limited!)

  ir remote:
  ----------

  If a TSOP38238 is connected to pin 5, you can also use an ir remote to remote
  control TonUINO. There are code mappings for the silver apple remote below, but you can
  change them to other codes to match different remotes. This feature can be enabled by
  uncommenting the define TSOP38238 below.

  There is one function, currently only available with the ir remote: Box lock.
  When TonUINO is locked, the buttons on TonUINO as well as the nfc reader are disabled
  until TonUINO is unlocked again. Playback continues while TonUINO is locked.

  During playback:
  center - toggle box lock
  play+pause - toggle playback
  up / down - volume up / down
  left / right - previous / next track in album, party and story book mode
  menu - reset progress to track 1 in story book mode

  During idle:
  center - toggle box lock
  menu - enter erase nfc tag mode

  During erase nfc tag mode:
  menu - cancel

  During nfc tag setup mode:
  play+pause - confirm
  right - next option
  left - previous option
  up - jump 10 folders or 10 tracks forward
  down - jump 10 folders or 10 tracks backwards
  center - announce folder number or track number
  menu - cancel

  status led:
  -----------

  If a led is connected to pin 6, limited status information is given using that led.
  The led is solid on when TonUINO is playing a track and it is pulsing slowly when
  TonUINO is idle. When TonUINO is in setup new nfc tag or erase nfc tag mode, the
  led is blinking every 500ms. And last but not not least, the led bursts 4 times when
  TonUINO is locked or unlocked. This feature can be enabled by uncommenting the
  define STATUSLED below.

  cubiekid:
  ---------

  If you happen to have a CubieKid case and the additional circuit board, this firmware
  supports both shutdown methods - due to low battery voltage as well as due to
  inactivity after a configurable ammount of time. The shutdown voltage, inactivity time
  as well as other parameters can be setup in the configuration section of this firmware.
  This feature can be enabled by uncommenting the define CUBIEKID below.

  The CubieKid case as well as the additional circuit board, have been designed and developed
  by Jens Hackel aka DB3JHF and can be found here: https://www.thingiverse.com/thing:3148200

  data stored on the nfc tags:
  ----------------------------

  Up to 16 bytes of data are stored in sector 1 / block 4, of which the first 8 bytes
  are currently in use.

  13 37 B3 47 01 02 05 11 00 00 00 00 00 00 00 00
  ----------- -- -- -- --
       |      |  |  |  |
       |      |  |  |  + assigned track (0x01-0xFF, only used in single mode)
       |      |  |  + assigned mode (0x01-0x05)
       |      |  + assigned folder (0x01-0x63)
       |      + version (currently always 0x01)
       + magic cookie to recognize that a card belongs to TonUINO

  non standard libraries used in this firmware:
  ---------------------------------------------

  MFRC522.h - https://github.com/miguelbalboa/rfid
  DFMiniMp3.h - https://github.com/Makuna/DFMiniMp3
  AceButton.h - https://github.com/bxparks/AceButton
  IRremote.h - https://github.com/z3t0/Arduino-IRremote
  Countimer.h - https://github.com/inflop/Countimer
  Vcc.h - https://github.com/Yveaux/Arduino_Vcc
*/

// uncomment the below line to enable ir remote support
// #define TSOP38238

// uncomment the below line to enable status led support
// #define STATUSLED

// uncomment the below line to enable cubiekid support
#define CUBIEKID

// uncomment the below line to enable additional cubiekid debug output
// #define CUBIEKIDDEBUG

// include required libraries
#include <SoftwareSerial.h>
#include <SPI.h>
#include <EEPROM.h>
#include <MFRC522.h>
#include <DFMiniMp3.h>
#include <AceButton.h>
using namespace ace_button;

// include additional library if ir remote support is enabled
#if defined(TSOP38238)
#include <IRremote.h>
#endif

// include additional libraries if cubiekid support is enabled
#if defined(CUBIEKID)
#include <Countimer.h>
#include <Vcc.h>
#endif

// define global constants
const uint8_t softwareSerialTxPin = 3;              // software serial tx, wired with 1k ohm to rx pin of DFPlayer Mini
const uint8_t softwareSerialRxPin = 2;              // software serial rx, wired straight to tx pin of DFPlayer Mini
const uint8_t mp3BusyPin = 4;                       // reports play state of DFPlayer Mini (LOW = playing)
const uint8_t irReceiverPin = 5;                    // pin used for the ir receiver
const uint8_t statusLedPin = 6;                     // pin used for status led
const uint8_t nfcResetPin = 9;                      // used for spi communication to nfc module
const uint8_t nfcSlaveSelectPin = 10;               // used for spi communication to nfc module
const uint8_t rngSeedPin = A0;                      // used to seed the random number generator
const uint8_t mp3StartVolume = 22;                  // initial volume of DFPlayer Mini
const uint8_t mp3MinVolume = 4;                     // minimal volume of DFPlayer Mini
const uint8_t mp3MaxVolume = 30;                    // maximal volume of DFPlayer Mini
//const Case1 increaseVolume = +3;                  // volume up of DFPlayer Mini
//const Case2 decreaseVolume = -3;                  // volume down of DFPlayer Mini
//const Case3 setVolume = 5;                        // set volume of DFPlayer Mini
const uint8_t button0Pin = A0;                      // middle button
const uint8_t button1Pin = A1;                      // right button
const uint8_t button2Pin = A2;                      // left button
const uint16_t buttonClickDelay = 1000;             // time during which a button click is still a click (in milliseconds)
const uint16_t buttonShortLongPressDelay = 2000;    // time after which a button press is considered a long press (in milliseconds)
const uint16_t buttonLongLongPressDelay = 5000;     // longer long press delay for special cases, i.e. to trigger erase nfc tag mode (in milliseconds)
const uint16_t ledBlinkInterval = 500;              // led blink interval (in milliseconds)
const uint32_t debugConsoleSpeed = 115200;          // speed for the debug console
uint8_t temp_playbackMode = 0;                      // used for admin functions
uint8_t temp_assignedFolder = 0;                    // used for admin functions
uint8_t temp_assignedTrack = 0;                     // used for admin functions
uint8_t temp_assignedVolume = 0;                     // used for admin functions
uint8_t temp_assignedSleeptime = 0;                  // used for admin functions
uint8_t sleeptimer_done = 0;                    	// used for admin functions
uint8_t sleeptimer_active = 0;                   	// used for admin functions

// define message to mp3 file mappings for spoken feedback
const uint16_t msgSetupNewTag = 305;                // 01
const uint16_t msgSetupNewTagFolderAssigned = 306;  // 02
const uint16_t msgSetupNewTagStoryMode = 310;       // 03
const uint16_t msgSetupNewTagAlbumMode = 320;       // 04
const uint16_t msgSetupNewTagPartyMode = 330;       // 05
const uint16_t msgSetupNewTagSingleMode = 340;      // 06
const uint16_t msgSetupNewTagSingleModeCont = 341;  // 07
const uint16_t msgSetupNewTagStoryBookMode = 350;   // 08
const uint16_t msgSetupNewTagConfirm = 390;         // 09
const uint16_t msgSetupNewTagCancel = 391;          // 10
const uint16_t msgSetupNewTagError = 392;           // 11
const uint16_t msgEraseTag = 405;                   // 12
const uint16_t msgEraseTagConfirm = 490;            // 13
const uint16_t msgEraseTagCancel = 491;             // 14
const uint16_t msgEraseTagError = 492;              // 15
const uint16_t msgWelcome = 505;                    // 16
const uint16_t msgBatteryLow = 510;                 // 17

// used to calculate the total ammount of tracks on the sd card (510 + count from above)
const uint16_t msgCount = 527;

// define code mappings for silver apple tv 2 ir remote
const uint16_t ir1ButtonUp = 0x5057;
const uint16_t ir1ButtonDown = 0x3057;
const uint16_t ir1ButtonLeft = 0x9057;
const uint16_t ir1ButtonRight = 0x6057;
const uint16_t ir1ButtonCenter = 0x3A57;
const uint16_t ir1ButtonMenu = 0xC057;
const uint16_t ir1ButtonPlayPause = 0xFA57;

// define code mappings for silver apple tv 3 ir remote
const uint16_t ir2ButtonUp = 0x50BF;
const uint16_t ir2ButtonDown = 0x30BF;
const uint16_t ir2ButtonLeft = 0x90BF;
const uint16_t ir2ButtonRight = 0x60BF;
const uint16_t ir2ButtonCenter = 0x3ABF;
const uint16_t ir2ButtonMenu = 0xC0BF;
const uint16_t ir2ButtonPlayPause = 0xFABF;

// button actions
enum {NOACTION,
      B0P, B1P, B2P,
      B0H, B1H, B2H,
      B0D, B1D, B2D,
      IRU, IRD, IRL, IRR, IRC, IRM, IRP
     };

// button modes
enum {PAUSE, PLAY, CONFIG};

// this object stores nfc tag data
struct nfcTagObject {
  uint32_t cookie;
  uint8_t version;
  uint8_t assignedFolder;
  uint8_t playbackMode;
  uint8_t assignedTrack;
  uint8_t assignedVolume;
  uint8_t assignedSleeptime;
} nfcTag;

// this object tracks the playback state
struct playbackObject {
  bool firstTrack = true;
  bool queueMode = false;
  uint8_t playTrack = 1;
  uint8_t storedTrack = 1;
  uint8_t playList[255];
  uint8_t folderTrackCount = 0;
} playback;

#if defined(CUBIEKID)
// this object stores the cubiekid configuration
struct cubiekidObject {
  const uint8_t shutdownPin = 7;                    // pin used to shutdown the system
  const uint8_t powerOffHours = 0;                  // hours until shutdown
  const uint8_t powerOffMinutes = 5;                // minutes until shutdown
  const uint8_t powerOffSeconds = 0;                // seconds until shutdown
  const uint16_t timerInterval = 1000;              // timer interval (in milliseconds)
  const float minVoltage = 1.0;                     // minimum expected voltage level (in volts)
  const float maxVoltage = 5.0;                     // maximum expected voltage level (in volts)
  const float voltageCorrection = 1.0 / 1.0;        // voltage measured by multimeter divided by reported voltage
} cubiekid;
#endif

// global variables
uint8_t inputEvent = NOACTION;

// ############################################################### no configuration below this line ###############################################################

// this function needs to be declared here for the first time because it's called in class Mp3Notify
// the function itself is down below
void playNextTrack(uint16_t globalTrack, bool directionForward, bool triggeredManually);

// used by DFPlayer Mini library during callbacks
class Mp3Notify {
  public:
    static void OnError(uint16_t returnValue) {
      switch (returnValue) {
        case DfMp3_Error_Busy:
          Serial.print(F("busy"));
          break;
        case DfMp3_Error_Sleeping:
          Serial.print(F("sleep"));
          break;
        case DfMp3_Error_SerialWrongStack:
          Serial.print(F("serial stack"));
          break;
        case DfMp3_Error_CheckSumNotMatch:
          Serial.print(F("checksum"));
          break;
        case DfMp3_Error_FileIndexOut:
          Serial.print(F("file index"));
          break;
        case DfMp3_Error_FileMismatch:
          Serial.print(F("file mismatch"));
          break;
        case DfMp3_Error_Advertise:
          Serial.print(F("advertise"));
          break;
        case DfMp3_Error_General:
          Serial.print(F("general"));
          break;
        default:
          Serial.print(F("unknown"));
          break;
      }
      Serial.print(F(" error: "));
      Serial.println(returnValue);
    }
    static void OnPlayFinished(uint16_t returnValue) {
      playNextTrack(returnValue, true, false);
    }
    static void OnCardOnline(uint16_t returnValue) {
      Serial.print(F("sd online: "));
      Serial.println(returnValue);
    }
    static void OnCardInserted(uint16_t returnValue) {
      Serial.print(F("sd inserted: "));
      Serial.println(returnValue);
    }
    static void OnCardRemoved(uint16_t returnValue) {
      Serial.print(F("sd removed: "));
      Serial.println(returnValue);
    }
};

SoftwareSerial secondarySerial(softwareSerialRxPin, softwareSerialTxPin);     // create SoftwareSerial instance
MFRC522 mfrc522(nfcSlaveSelectPin, nfcResetPin);                              // create MFRC522 instance
DFMiniMp3<SoftwareSerial, Mp3Notify> mp3(secondarySerial);                    // create DFMiniMp3 instance
ButtonConfig button0Config;                                                   // create ButtonConfig instance
ButtonConfig button1Config;                                                   // create ButtonConfig instance
ButtonConfig button2Config;                                                   // create ButtonConfig instance
AceButton button0(&button0Config);                                            // create AceButton instance
AceButton button1(&button1Config);                                            // create AceButton instance
AceButton button2(&button2Config);                                            // create AceButton instance

#if defined(TSOP38238)
IRrecv irReceiver(irReceiverPin);                                             // create IRrecv instance
decode_results irReadings;                                                    // create decode_results instance to store received ir readings
#endif

#if defined(CUBIEKID)
Countimer cubiekidShutdownTimer;                                              // create Countimer instance
Countimer sleepTimer;                                      			          // create Countimer instance
Vcc cubiekidVoltage(cubiekid.voltageCorrection);                              // create Vcc instance
#endif

// checks all input sources and populates the global inputEvent variable
void checkForInput() {
  // clear inputEvent
  inputEvent = 0;

  // check all buttons
  button0.check();
  button1.check();
  button2.check();

#if defined(TSOP38238)
  // poll ir receiver, has precedence over (overwrites) physical buttons
  if (irReceiver.decode(&irReadings)) {
    switch (irReadings.value & 0xFFFF) {
      // button up
      case ir1ButtonUp:
      case ir2ButtonUp:
        inputEvent = IRU;
        break;
      // button down
      case ir1ButtonDown:
      case ir2ButtonDown:
        inputEvent = IRD;
        break;
      // button left
      case ir1ButtonLeft:
      case ir2ButtonLeft:
        inputEvent = IRL;
        break;
      // button right
      case ir1ButtonRight:
      case ir2ButtonRight:
        inputEvent = IRR;
        break;
      // button center
      case ir1ButtonCenter:
      case ir2ButtonCenter:
        inputEvent = IRC;
        break;
      // button menu
      case ir1ButtonMenu:
      case ir2ButtonMenu:
        inputEvent = IRM;
        break;
      // button play+pause
      case ir1ButtonPlayPause:
      case ir2ButtonPlayPause:
        inputEvent = IRP;
        break;
      default:
        break;
    }
    irReceiver.resume();
  }
#endif
}

// translates the various button events into enums
void translateButtonInput(AceButton* button, uint8_t eventType, uint8_t /* buttonState */) {
  switch (button->getId()) {
    // botton 0 (middle)
    case 0:
      switch (eventType) {
        case AceButton::kEventClicked:
          inputEvent = B0P;
          break;
        case AceButton::kEventLongPressed:
          inputEvent = B0H;
          break;
        case AceButton::kEventDoubleClicked:
          inputEvent = B0D;
          break;
        default:
          break;
      }
      break;
    // botton 1 (right)
    case 1:
      switch (eventType) {
        case AceButton::kEventClicked:
          inputEvent = B1P;
          break;
        case AceButton::kEventLongPressed:
          inputEvent = B1H;
          break;
        default:
          break;
      }
      break;
    // botton 2 (left)
    case 2:
      switch (eventType) {
        case AceButton::kEventClicked:
          inputEvent = B2P;
          break;
        case AceButton::kEventLongPressed:
          inputEvent = B2H;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

// switches button configuration dependig on the state that TonUINO is in
void switchButtonConfiguration(uint8_t buttonMode) {
  // default configuration for all modes
  // button 0
  button0Config.setEventHandler(translateButtonInput);
  button0Config.setFeature(ButtonConfig::kFeatureClick);
  button0Config.setFeature(ButtonConfig::kFeatureLongPress);
  button0Config.setClickDelay(buttonClickDelay);
  button0Config.setLongPressDelay(buttonShortLongPressDelay);
  // button 1
  button1Config.setEventHandler(translateButtonInput);
  button1Config.setFeature(ButtonConfig::kFeatureClick);
  button1Config.setFeature(ButtonConfig::kFeatureLongPress);
  button1Config.setClickDelay(buttonClickDelay);
  button1Config.setLongPressDelay(buttonShortLongPressDelay);
  // button 2
  button2Config.setEventHandler(translateButtonInput);
  button2Config.setFeature(ButtonConfig::kFeatureClick);
  button2Config.setFeature(ButtonConfig::kFeatureLongPress);
  button2Config.setClickDelay(buttonClickDelay);
  button2Config.setLongPressDelay(buttonShortLongPressDelay);

  // non default configuration
  switch (buttonMode) {
    case PAUSE:
      // button 0
      button0Config.clearFeature(ButtonConfig::kFeatureDoubleClick);
      button0Config.clearFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
      button0Config.setLongPressDelay(buttonLongLongPressDelay);
      break;
    case PLAY:
      // button 0
      button0Config.clearFeature(ButtonConfig::kFeatureDoubleClick);
      button0Config.clearFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
      button0Config.setLongPressDelay(buttonLongLongPressDelay);
      break;
    case CONFIG:
      // button 0
      button0Config.setFeature(ButtonConfig::kFeatureDoubleClick);
      button0Config.setFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
      button0Config.setLongPressDelay(buttonShortLongPressDelay);
      break;
    default:
      break;
  }
}

// waits for current playing track to finish
void waitPlaybackToFinish() {
  uint64_t waitPlaybackToStartMillis = millis();
  delay(100);
  while (digitalRead(mp3BusyPin)) if (millis() - waitPlaybackToStartMillis >= 10000) break;
  while (!digitalRead(mp3BusyPin));
}

// prints current mode, folder and track information
void printModeFolderTrack(uint8_t playTrack, bool cr) {
  if (nfcTag.playbackMode == 1) Serial.print(F("story > "));
  else if (nfcTag.playbackMode == 2) Serial.print(F("album > "));
  else if (nfcTag.playbackMode == 3) Serial.print(F("party > "));
  else if (nfcTag.playbackMode == 4) Serial.print(F("single > "));
  else if (nfcTag.playbackMode == 5) Serial.print(F("story book > "));
  Serial.print(nfcTag.assignedFolder);
  Serial.print(F("|"));
  Serial.print(playTrack);
  Serial.print(F("/"));
  Serial.print(playback.folderTrackCount);
  if (cr) Serial.println();
}

// plays next track depending on the current playback mode
void playNextTrack(uint16_t globalTrack, bool directionForward, bool triggeredManually) {
  static uint16_t lastCallTrack = 0;

  //delay 100ms to be on the safe side with the serial communication
  delay(100);

  
  if(sleeptimer_done == 1){
	  cubiekidShutdown();
  }
  
  
  // we only advance to a new track when in queue mode, not during interactive prompt playback (ie. during configuration of a new nfc tag)
  if (!playback.queueMode) return;

  // story mode (1): play one random track in folder
  // single mode (4): play one single track in folder
  // there is no next track in story and single mode, stop playback
  if (nfcTag.playbackMode == 1 || nfcTag.playbackMode == 4) {
    playback.queueMode = false;
    switchButtonConfiguration(PAUSE);
#if defined(CUBIEKID)
    cubiekidShutdownTimer.restart();
#endif
    if (nfcTag.playbackMode == 1) Serial.print(F("story"));
    else if (nfcTag.playbackMode == 4) Serial.print(F("single"));
    Serial.println(F(" > stop"));
    mp3.stop();
  }

  // album mode (2): play the complete folder
  // party mode (3): shuffle the complete folder
  // story book mode (5): play the complete folder and track progress
  // advance to the next or previous track, stop if the end of the folder is reached
  if (nfcTag.playbackMode == 2 || nfcTag.playbackMode == 3 || nfcTag.playbackMode == 5) {

    // **workaround for some DFPlayer mini modules that make two callbacks in a row when finishing a track**
    // reset lastCallTrack to avoid lockup when playback was just started
    if (playback.firstTrack) {
      playback.firstTrack = false;
      lastCallTrack = 0;
    }
    // check if we get called with the same track number twice in a row, if yes return immediately
    if (lastCallTrack == globalTrack) return;
    else lastCallTrack = globalTrack;

    // play next track?
    if (directionForward) {
      // there are more tracks after the current one, play next track
      if (playback.playTrack < playback.folderTrackCount) {
        playback.playTrack++;
        if (nfcTag.playbackMode == 2 || nfcTag.playbackMode == 5) {
          printModeFolderTrack(playback.playTrack, true);
          mp3.playFolderTrack(nfcTag.assignedFolder, playback.playTrack);
        }
        else if (nfcTag.playbackMode == 3) {
          printModeFolderTrack(playback.playList[playback.playTrack - 1], true);
          mp3.playFolderTrack(nfcTag.assignedFolder, playback.playList[playback.playTrack - 1]);
        }
      }
      // there are no more tracks after the current one
      else {
        // if not triggered manually, stop playback (and reset progress)
        if (!triggeredManually) {
          playback.queueMode = false;
          switchButtonConfiguration(PAUSE);
#if defined(CUBIEKID)
          cubiekidShutdownTimer.restart();
#endif
          if (nfcTag.playbackMode == 2) Serial.println(F("album > stop"));
          else if (nfcTag.playbackMode == 3) Serial.println(F("party > stop"));
          else if (nfcTag.playbackMode == 5) {
            Serial.println(F("story book > stop+reset"));
            EEPROM.update(nfcTag.assignedFolder, 0);
          }
          mp3.stop();
        }
      }
    }
    // play previous track?
    else {
      // there are more tracks before the current one, play the previous track
      if (playback.playTrack > 1) {
        playback.playTrack--;
        if (nfcTag.playbackMode == 2 || nfcTag.playbackMode == 5) {
          printModeFolderTrack(playback.playTrack, true);
          mp3.playFolderTrack(nfcTag.assignedFolder, playback.playTrack);
        }
        else if (nfcTag.playbackMode == 3) {
          printModeFolderTrack(playback.playList[playback.playTrack - 1], true);
          mp3.playFolderTrack(nfcTag.assignedFolder, playback.playList[playback.playTrack - 1]);
        }
      }
    }
  }
}

// reads data from nfc tag
uint8_t readNfcTagData() {
  uint8_t returnCode;
  uint8_t mifareBlock = 4;
  uint8_t mifareData[18];
  uint8_t mifareDataSize = sizeof(mifareData);
  MFRC522::StatusCode mifareStatus;
  MFRC522::MIFARE_Key mifareKey;
  MFRC522::PICC_Type mifareType;
  for (uint8_t i = 0; i < 6; i++) mifareKey.keyByte[i] = 0xFF;

  // check if card type is supported
  mifareType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  if (mifareType != MFRC522::PICC_TYPE_MIFARE_MINI && mifareType != MFRC522::PICC_TYPE_MIFARE_1K && mifareType != MFRC522::PICC_TYPE_MIFARE_4K) {
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    returnCode = 255;
  }
  else {
    // check if we can authenticate with mifareKey
    mifareStatus = (MFRC522::StatusCode)mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, mifareBlock, &mifareKey, &(mfrc522.uid));
    if (mifareStatus != MFRC522::STATUS_OK) {
      mfrc522.PICC_HaltA();
      mfrc522.PCD_StopCrypto1();
      returnCode = 254;
    }
    else {
      // read data from nfc tag
      mifareStatus = (MFRC522::StatusCode)mfrc522.MIFARE_Read(mifareBlock, mifareData, &mifareDataSize);
      if (mifareStatus != MFRC522::STATUS_OK) {
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        returnCode = 253;
      }
      else {
        // log data to the console
        Serial.print(F("read"));
        for (uint8_t i = 0; i < 16; i++) {
          Serial.print(mifareData[i] < 0x10 ? " 0" : " ");
          Serial.print(mifareData[i], HEX);
        }
        Serial.println();

        // convert 4 byte cookie to 32bit decimal for easier handling
        uint32_t tempCookie;
        tempCookie  = (uint32_t)mifareData[0] << 24;
        tempCookie += (uint32_t)mifareData[1] << 16;
        tempCookie += (uint32_t)mifareData[2] << 8;
        tempCookie += (uint32_t)mifareData[3];

        // if cookie is not blank, update ncfTag object with data read from nfc tag
        if (tempCookie != 0) {
          mfrc522.PICC_HaltA();
          mfrc522.PCD_StopCrypto1();
          nfcTag.cookie = tempCookie;
          nfcTag.version = mifareData[4];
          nfcTag.assignedFolder = mifareData[5];
          nfcTag.playbackMode = mifareData[6];
          nfcTag.assignedTrack = mifareData[7];
          nfcTag.assignedVolume = mifareData[9];
          nfcTag.assignedSleeptime = mifareData[10];
        }
        // if cookie is blank, clear ncfTag object
        else {
          nfcTag.cookie = 0;
          nfcTag.version = 0;
          nfcTag.assignedFolder = 0;
          nfcTag.playbackMode = 0;
          nfcTag.assignedTrack = 0;
          nfcTag.assignedVolume = 0;
          nfcTag.assignedSleeptime = 0;
        }
        returnCode = 1;
      }
    }
  }

  return returnCode;
}


#if defined(STATUSLED)
// fade in/out status led while beeing idle, during playback set to full brightness
void fadeStatusLed(bool isPlaying) {
  static bool statusLedDirection = false;
  static int16_t statusLedValue = 255;
  static uint64_t statusLedOldMillis;

  // TonUINO is playing, set status led to full brightness
  if (isPlaying) {
    statusLedValue = 255;
    digitalWrite(statusLedPin, true);
  }
  // TonUINO is not playing, fade status led in/out
  else {
    uint64_t statusLedNewMillis = millis();
    if (statusLedNewMillis - statusLedOldMillis >= 100) {
      statusLedOldMillis = statusLedNewMillis;
      if (statusLedDirection) {
        statusLedValue += 10;
        if (statusLedValue >= 255) {
          statusLedValue = 255;
          statusLedDirection = !statusLedDirection;
        }
      }
      else {
        statusLedValue -= 10;
        if (statusLedValue <= 0) {
          statusLedValue = 0;
          statusLedDirection = !statusLedDirection;
        }
      }
      analogWrite(statusLedPin, statusLedValue);
    }
  }
}

// blink status led every blinkInterval milliseconds
void blinkStatusLed(uint16_t blinkInterval) {
  static bool statusLedState;
  static uint64_t statusLedOldMillis;

  uint64_t statusLedNewMillis = millis();
  if (statusLedNewMillis - statusLedOldMillis >= blinkInterval) {
    statusLedOldMillis = statusLedNewMillis;
    statusLedState = !statusLedState;
    digitalWrite(statusLedPin, statusLedState);
  }
}

// burst status led 4 times
void burstStatusLed() {
  bool statusLedState = true;

  for (uint8_t i = 0; i < 8; i++) {
    statusLedState = !statusLedState;
    digitalWrite(statusLedPin, statusLedState);
    delay(100);
  }
}
#endif

#if defined(CUBIEKID)
// show time remaining until shutdown
// available when CUBIEKIDDEBUG is enabled
void cubiekidTimerRemaining() {
#if defined(CUBIEKIDDEBUG)
  Serial.print(F("shutdown in "));
  Serial.println(cubiekidShutdownTimer.getCurrentTime());
#endif
}

void sleepTimerRemaining() {
#if defined(CUBIEKIDDEBUG)
  Serial.print(F("sleeptimer in "));
  Serial.println(sleepTimer.getCurrentTime());
#endif
}


// shutdown the system
void cubiekidShutdown() {
  Serial.println(F("inactivity shutdown..."));
  digitalWrite(cubiekid.shutdownPin, HIGH);
}

void sleepTimerFunction() {
  Serial.println(F("sleeptimer reached"));
  sleeptimer_done = 1;
}

#endif

void setup() {
#if defined(CUBIEKID)
  pinMode(cubiekid.shutdownPin, OUTPUT);
  digitalWrite(cubiekid.shutdownPin, LOW);
#endif
  Serial.begin(debugConsoleSpeed);
  while (!Serial);
  Serial.println(F("TonUINO JUKEBOX"));
  Serial.println(F("by Thorsten Voß"));
  Serial.println(F("Stephan Eisfeld"));
  Serial.println(F("---------------"));
  Serial.println(F("init nfc"));
  SPI.begin();
  mfrc522.PCD_Init();
  mfrc522.PCD_DumpVersionToSerial();

  Serial.println(F("init mp3"));
  mp3.begin();
  delay(2000);
  Serial.print(F("  volume "));
  Serial.println(mp3StartVolume);
  mp3.setVolume(mp3StartVolume);
  Serial.print(F("  tracks "));
  Serial.println(mp3.getTotalTrackCount() - msgCount);
  pinMode(mp3BusyPin, INPUT);

  Serial.println(F("init rng"));
  randomSeed(analogRead(rngSeedPin));

  Serial.println(F("init buttons"));
  pinMode(button0Pin, INPUT_PULLUP);
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  button0.init(button0Pin, HIGH, 0);
  button1.init(button1Pin, HIGH, 1);
  button2.init(button2Pin, HIGH, 2);
  switchButtonConfiguration(PAUSE);

#if defined(TSOP38238)
  Serial.println(F("init ir"));
  irReceiver.enableIRIn();
#endif

#if defined(STATUSLED)
  Serial.println(F("init led"));
  pinMode(statusLedPin, OUTPUT);
  digitalWrite(statusLedPin, HIGH);
#endif

#if defined(CUBIEKID)
  Serial.println(F("init cubiekid"));
  Serial.print(F("  expected "));
  Serial.print(cubiekid.maxVoltage);
  Serial.println(F("V"));
  Serial.print(F("  shutdown "));
  Serial.print(cubiekid.minVoltage);
  Serial.println(F("V"));
  Serial.print(F("   current "));
  Serial.print(cubiekidVoltage.Read_Volts());
  Serial.print(F("V ("));
  Serial.print(cubiekidVoltage.Read_Perc(cubiekid.minVoltage, cubiekid.maxVoltage));
  Serial.println(F("%)"));
  Serial.print(F("     timer "));
  Serial.print(cubiekid.powerOffHours);
  Serial.print(F("h:"));
  Serial.print(cubiekid.powerOffMinutes);
  Serial.print(F("m:"));
  Serial.print(cubiekid.powerOffSeconds);
  Serial.println(F("s"));
  cubiekidShutdownTimer.setCounter(cubiekid.powerOffHours, cubiekid.powerOffMinutes, cubiekid.powerOffSeconds, cubiekidShutdownTimer.COUNT_DOWN, cubiekidShutdown);
  cubiekidShutdownTimer.setInterval(cubiekidTimerRemaining, cubiekid.timerInterval);
  cubiekidShutdownTimer.start();
#endif

  // hold down all three buttons while powering up: erase the eeprom contents
  if (digitalRead(button0Pin) == LOW && digitalRead(button1Pin) == LOW && digitalRead(button2Pin) == LOW) {
    Serial.println(F("init eeprom"));
    for (uint16_t i = 0; i < EEPROM.length(); i++) {
      EEPROM.update(i, 0);
#if defined(STATUSLED)
      blinkStatusLed(50);
#endif
    }
  }

  Serial.println(F("system is ready"));
  mp3.playMp3FolderTrack(msgWelcome);
}

void increaseVolume(){
    uint8_t mp3CurrentVolume = mp3.getVolume();
    if (mp3CurrentVolume < mp3MaxVolume) {
      Serial.print(F("volume "));
      Serial.println(mp3CurrentVolume + 1);
      mp3.increaseVolume();
    }
    else {
      Serial.print(F("maximum volume "));
      Serial.println(mp3MaxVolume);
    }
}

void decreaseVolume(){
    uint8_t mp3CurrentVolume = mp3.getVolume();
    if (mp3CurrentVolume > 0) {
      if (mp3CurrentVolume <= mp3MinVolume) Serial.println(F("MinVolume"));
      else {
        Serial.print(F("volume "));
        Serial.println(mp3CurrentVolume - 1);
        mp3.decreaseVolume();
      }
    }
    else Serial.println(F("mute"));
}

void loop() {
  static bool isLocked = false;
  bool isPlaying = !digitalRead(mp3BusyPin);
  checkForInput();

digitalWrite(statusLedPin, true);

#if defined(STATUSLED)
  fadeStatusLed(isPlaying);
#endif

#if defined(CUBIEKID)
  // update timer
  cubiekidShutdownTimer.run();
  if(sleeptimer_active == 1){
	sleepTimer.run();
  }
  // if low voltage level is reached, store progress and shutdown
  if (cubiekidVoltage.Read_Volts() <= cubiekid.minVoltage) {
    Serial.print(cubiekidVoltage.Read_Volts());
    // if the current playback mode is story book mode: store the current progress
    if (nfcTag.playbackMode == 5) EEPROM.update(nfcTag.assignedFolder, playback.playTrack);
    mp3.playMp3FolderTrack(msgBatteryLow);
    waitPlaybackToFinish();
    Serial.println(F("-low voltage shutdown..."));
    digitalWrite(cubiekid.shutdownPin, LOW);
  }
#endif

  // ################################################################################
  // # main code block, if nfc tag is detected and TonUINO is not locked do something
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial() && !isLocked) {
    // if the current playback mode is story book mode, only while playing: store the current progress
    if (nfcTag.playbackMode == 5 && isPlaying) {
      printModeFolderTrack(playback.playTrack, false);
      Serial.println(F(" > saved"));
      EEPROM.update(nfcTag.assignedFolder, playback.playTrack);
    }

  temp_playbackMode = nfcTag.playbackMode;
  temp_assignedFolder = nfcTag.assignedFolder;
  temp_assignedTrack = nfcTag.assignedTrack;
  temp_assignedVolume = nfcTag.assignedVolume;
  temp_assignedSleeptime = nfcTag.assignedSleeptime;
  
  uint8_t readNfcTagStatus = readNfcTagData();
    // ##############################
    // # nfc tag is successfully read
  if (readNfcTagStatus == 1) {
      // #######################################################################################################
      // # nfc tag has our magic cookie 0x1337 0xb347 on it (322417479), use data from nfc tag to start playback
      if (nfcTag.cookie == 322417479) {
        switchButtonConfiguration(PLAY);

        // log data to the console
        Serial.print(F("read version "));
        Serial.print(nfcTag.version);
        Serial.print(F(" | folder "));
        Serial.print(nfcTag.assignedFolder);
        Serial.print(F(" | mode "));
        switch (nfcTag.playbackMode) {
          case 1:
            Serial.println(F("1 (story)"));
            break;
          case 2:
            Serial.println(F("2 (album)"));
            break;
          case 3:
            Serial.println(F("3 (party)"));
            break;
          case 4:
            Serial.print(F("4 (single, track "));
            Serial.print(nfcTag.assignedTrack);
            Serial.println(F(")"));
            break;
          case 5:
            Serial.println(F("5 (story book)"));
            break;
          case 6:
            Serial.println(F("6 (admin)"));

			  //nfcTag.assignedTrack
			  //nfcTag.assignedFolder
			  Serial.print(F("admin mode > "));

			  switch (nfcTag.assignedTrack) {
				case 1:
				  for (int i=0; i<nfcTag.assignedFolder; i++){
				  increaseVolume();
				  }
				break;
				case 2:
				  for (int i=0; i<nfcTag.assignedFolder; i++){
				  decreaseVolume();
				  }
				break;
				case 3:
				  mp3.setVolume(nfcTag.assignedFolder);
				  Serial.print(F("volume "));
				  Serial.println(nfcTag.assignedFolder);
				break;
				case 4:
				  inputEvent = B0P;
				break;
				case 5:
				  inputEvent = B2H;
				break;
				case 6:
				  inputEvent = B1H;
				break;
				case 7:
				  Serial.print(F("sleep timer"));
				  sleepTimer.setCounter(0, nfcTag.assignedFolder, 0, sleepTimer.COUNT_DOWN, sleepTimerFunction);
				  sleepTimer.setInterval(sleepTimerRemaining, 1000);
				  sleepTimer.run();
				  sleepTimer.start();
				  sleeptimer_active = 1;
				break;
				  
				  

				default:
				break;
			  }
			  
			  break;
		  
	  default:
	  break;
	}
    
	if (nfcTag.assignedVolume != 0){
	  mp3.setVolume(nfcTag.assignedVolume);
	  Serial.print(F("volume "));
	  Serial.println(nfcTag.assignedVolume);
	}
	if (nfcTag.assignedSleeptime != 0){
	  Serial.print(F("sleep timer"));
	  sleepTimer.setCounter(0, nfcTag.assignedSleeptime, 0, sleepTimer.COUNT_DOWN, sleepTimerFunction);
	  sleepTimer.setInterval(sleepTimerRemaining, 1000);
	  sleepTimer.run();
	  sleepTimer.start();
	  sleeptimer_active = 1;
	}
    
    if(nfcTag.playbackMode != 6){
      
      #if defined(CUBIEKID)
        cubiekidShutdownTimer.stop();
      #endif
      // start playback
      playback.folderTrackCount = mp3.getFolderTrackCount(nfcTag.assignedFolder);
      switch (nfcTag.playbackMode) {
		  // story mode
		  case 1:
		  playback.playTrack = random(1, playback.folderTrackCount + 1);
		  printModeFolderTrack(playback.playTrack, true);
		  break;
		  // album mode
		  case 2:
		  playback.playTrack = 1;
		  printModeFolderTrack(playback.playTrack, true);
		  break;
		  // party mode
		  case 3:
		  playback.playTrack = 1;
		  // fill playlist
		  for (uint8_t i = 0; i < 255; i++) playback.playList[i] = i + 1 <= playback.folderTrackCount ? i + 1 : 0;
		  // shuffle playlist
		  for (uint8_t i = 0; i < playback.folderTrackCount; i++) {
		  uint8_t j = random(0, playback.folderTrackCount);
		  uint8_t temp = playback.playList[i];
		  playback.playList[i] = playback.playList[j];
		  playback.playList[j] = temp;
		  }
		  printModeFolderTrack(playback.playList[playback.playTrack - 1], true);
		  break;
		  // single mode
		  case 4:
		  playback.playTrack = nfcTag.assignedTrack;
		  printModeFolderTrack(playback.playTrack, true);
		  break;
		  // story book mode
		  case 5:
		  playback.playTrack = 1;
		  playback.storedTrack = EEPROM.read(nfcTag.assignedFolder);
		  // don't resume from eeprom, play from the beginning
		  if (playback.storedTrack == 0 || playback.storedTrack > playback.folderTrackCount) playback.playTrack = 1;
		  // resume from eeprom
		  else {
			playback.playTrack = playback.storedTrack;
			Serial.print(F("resuming "));
		  }
		  printModeFolderTrack(playback.playTrack, true);
		  break;
		  
		  default:
		  break;
      }
    
      playback.firstTrack = true;
      playback.queueMode = true;
		if (nfcTag.playbackMode == 1) mp3.playFolderTrack(nfcTag.assignedFolder, playback.playTrack);
		if (nfcTag.playbackMode == 2) mp3.playFolderTrack(nfcTag.assignedFolder, playback.playTrack);
		if (nfcTag.playbackMode == 3) mp3.playFolderTrack(nfcTag.assignedFolder, playback.playList[playback.playTrack - 1]);
		if (nfcTag.playbackMode == 4) mp3.playFolderTrack(nfcTag.assignedFolder, playback.playTrack);
		if (nfcTag.playbackMode == 5) mp3.playFolderTrack(nfcTag.assignedFolder, playback.playTrack);
	  } else {
		  nfcTag.playbackMode = temp_playbackMode;
		  nfcTag.assignedFolder = temp_assignedFolder;
		  nfcTag.assignedTrack = temp_assignedTrack;
		  nfcTag.assignedVolume = temp_assignedVolume;
		  nfcTag.assignedSleeptime = temp_assignedSleeptime;
    }
    
  }

      // nfc tag is not blank but unknown, ignore
      else Serial.println(F("tag is not one of ours"));
      // # end - nfc tag does not have our magic cookie 0x1337 0xb347 on it (0)
      // ######################################################################
    }
    // # end - nfc tag is successfully read
    // ####################################

    // handle errors that happened during reading from the nfc tag
    else if (readNfcTagStatus == 253) Serial.println(F("read error"));
    else if (readNfcTagStatus == 254) Serial.println(F("authentication failed"));
    else if (readNfcTagStatus == 255) Serial.println(F("tag is not supported"));
    else Serial.println(F("unknown error"));
  }
  // # end - main code block
  // #######################

  // ##################################################################################
  // # handle button and ir remote events during playback or while waiting for nfc tags
  // ir remote center: toggle box lock
  if (inputEvent == IRC) {
    if (!isLocked) {
      Serial.println(F("box locked"));
      isLocked = true;
#if defined(STATUSLED)
      burstStatusLed();
#endif
    }
    else {
      Serial.println(F("box unlocked"));
      isLocked = false;
#if defined(STATUSLED)
      burstStatusLed();
#endif
    }
  }
  // button 0 (middle) press or ir remote play+pause: toggle playback
  else if ((inputEvent == B0P && !isLocked) || inputEvent == IRP) {
    if (isPlaying) {
      switchButtonConfiguration(PAUSE);
#if defined(CUBIEKID)
      cubiekidShutdownTimer.start();
#endif
      Serial.println(F("pause"));
      mp3.pause();
      // if the current playback mode is story book mode: store the current progress
      if (nfcTag.playbackMode == 5) {
        printModeFolderTrack(playback.playTrack, false);
        Serial.println(F(" > saved"));
        EEPROM.update(nfcTag.assignedFolder, playback.playTrack);
      }
    }
    else {
      if (playback.queueMode) {
        switchButtonConfiguration(PLAY);
#if defined(CUBIEKID)
        cubiekidShutdownTimer.stop();
#endif
        Serial.println(F("play"));
        mp3.start();
      }
    }
  }
  // button 1 (right) press or ir remote up while playing: increase volume
  else if (((inputEvent == B1P && !isLocked) || inputEvent == IRU) && isPlaying) {
    increaseVolume();
  }
  // button 2 (left) press or ir remote down while playing: decrease volume
  else if (((inputEvent == B2P && !isLocked) || inputEvent == IRD) && isPlaying) {
    decreaseVolume();
  }
  // button 1 (right) hold for 2 sec or ir remote right, only during album, party and story book mode while playing: next track
  else if (((inputEvent == B1H && !isLocked) || inputEvent == IRR) && (nfcTag.playbackMode == 2 || nfcTag.playbackMode == 3 || nfcTag.playbackMode == 5) && isPlaying) {
    Serial.println(F("next track"));
    playNextTrack(random(65536), true, true);
  }
  // button 2 (left) hold for 2 sec or ir remote left, only during album, party and story book mode while playing: previous track
  else if (((inputEvent == B2H && !isLocked) || inputEvent == IRL) && (nfcTag.playbackMode == 2 || nfcTag.playbackMode == 3 || nfcTag.playbackMode == 5) && isPlaying) {
    Serial.println(F("previous track"));
    playNextTrack(random(65536), false, true);
  }
  // button 0 (middle) hold for 5 sec or ir remote menu, only during story book mode while playing: reset progress
  else if (((inputEvent == B0H && !isLocked) || inputEvent == IRM) && nfcTag.playbackMode == 5 && isPlaying) {
    playback.playTrack = 1;
    printModeFolderTrack(playback.playTrack, false);
    Serial.println(F(" > reset"));
    EEPROM.update(nfcTag.assignedFolder, 0);
    mp3.playFolderTrack(nfcTag.assignedFolder, playback.playTrack);
  }
  // # end - handle button or ir remote events during playback or while waiting for nfc tags
  // #######################################################################################

  mp3.loop();
}
