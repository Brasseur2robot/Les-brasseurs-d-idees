/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <ArduinoJson.h>
#include <SD.h>
#include "config.h"
#include "robot_mgr.h"
#include "sdcard.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ROBOT_MGR_DEBUG false

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the robot_mgr module.

   @param     none

   @result    none

*/
void RobotMgrInit()
{
  const char* filename = "/test.json"; 

  /* Takes the file from the SD file system */
  bool result_b = RobotMgrLoadConfiguration(filename);
  
  /* is it reeeaaaady to rumble ? */
  Serial.print("RobotMgr|Init : ");
  if (result_b == true)
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("FAILED");
  }
}

void RobotMgrUpdate()
{
  /* Takes the next step in the json object */

  /* if xy, goes to, and wait for arrival */

  /* if action, launches the action and looks if it has to wait for the end or not ? */

}

bool RobotMgrLoadConfiguration(const char* filename) //, Config& config) {
{
  bool result_b = false;

  /* Open file for reading */
  File file = SdcardReadFile(SD, "/test.json");

  if (!file) {
    Serial.println("RobotMgr|Failed to open config file for reading");
  }

  /* Allocate a temporary JsonDocument */
  JsonDocument doc;

  /* Deserialize the JSON document */
  DeserializationError error = deserializeJson(doc, file);
  if (error)
  {
    Serial.println("RobotMgr|Failed to read file");
    /* TODO Here should switch on a default trajectory */
  }

  JsonArray array = doc.as<JsonArray>();

  /* Create the pointer to the trajectory variable */
  pose_t * trajectoryJsonPoseArray_pst;
  /* Dynamically allocate the size */
  trajectoryJsonPoseArray_pst = (pose_t*)malloc( array.size() * sizeof( pose_t) );
  if (trajectoryJsonPoseArray_pst == NULL)
  {
    Serial.println("RobotMgr|Error trying to allocate memory space for the traj data.");
  }
  else
  {
    /* everything happened, should be ok */
    result_b = true;
  }

  uint8_t idx = 0;
  for(JsonVariant v : array) {
    /* Each line of the array is made of 7 objects */
    JsonObject object = v.as<JsonObject>();
    trajectoryJsonPoseArray_pst[idx].x = object["x"];
    trajectoryJsonPoseArray_pst[idx].y = object["y"];
    trajectoryJsonPoseArray_pst[idx].theta = object["angle"];
    trajectoryJsonPoseArray_pst[idx].resetTheta = object["orientation"];
    trajectoryJsonPoseArray_pst[idx].direction = object["direction"];
    trajectoryJsonPoseArray_pst[idx].clawState_u8 = object["action"];
    bool wea = object["wea"];
    idx++;
  }

  if(ROBOT_MGR_DEBUG)
  {
    Serial.print("RobotMgr|The number of poses is : ");
    Serial.print(array.size());
    Serial.println();
    for (uint8_t idx; idx < array.size(); idx++)
    {
      Serial.print("RobotMgr|Pose nb :");
      Serial.print(idx);
      Serial.print(", x : ");
      Serial.print(trajectoryJsonPoseArray_pst[idx].x);
      Serial.print(", y : ");
      Serial.print(trajectoryJsonPoseArray_pst[idx].y);
      Serial.print(", angle : ");
      Serial.print(trajectoryJsonPoseArray_pst[idx].theta);
      Serial.print(", orientation/resetTheta : ");
      Serial.print(trajectoryJsonPoseArray_pst[idx].resetTheta);
      Serial.print(", direction : ");
      Serial.print(trajectoryJsonPoseArray_pst[idx].direction);
      Serial.print(", action : ");
      Serial.print(trajectoryJsonPoseArray_pst[idx].clawState_u8);
      //Serial.print(", wea : ");
      //Serial.print(wea);
      Serial.println();
    }
  }
  
  /* Close the file */
  SdCardCloseFile(SD, file);

  return result_b;
}
