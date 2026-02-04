#include "driverSD.hpp"

//defining fileName here to be used in multiple functions
const char* fileName;
float32_t dataBuffer [40][10];
int numBufferCol;
int dataBufferIndex = 0;

driverSD::driverSD(const char* fileRoot, int numOfColumns){
    findCurrentFileName(fileRoot);
    numBufferCol = numOfColumns;
}

//accessers
const char* driverSD::getCurrentFileName(){
    return fileName;
}

int driverSD::getCurrentIndex(){
    return dataBufferIndex;
}

//mutators
void driverSD::increaseCurrentIndexBy(int increment){
    dataBufferIndex = dataBufferIndex + increment;
}

//this function will loop though the file names in SD card and get the file name of this 
//iteration to be something different to not override.

void driverSD::findCurrentFileName(String fileRoot){
    //intially setting file number to zero
    int fileNumber = 0;

    //going through the SD card, checking if the file name exists. If it does, incrementing the file number and try again
    while (SD.exists(fileRoot.c_str())){
        fileNumber++;
        fileRoot = fileRoot + String(fileNumber)+ ".txt";
    }

    fileName = fileRoot.c_str();
}

//this function will add landed data to the data buffer
//this method will take save the orientation of x, y, z to an array. After 40 sets have been collected, those sets are printed to file
//on sd card and array rows are overwrittten with new data.
  void driverSD::addLandedData(sh2_SensorValue_t event, File dataFile){
    //adding data to array
      dataBuffer[dataBufferIndex][0] = (float32_t)millis();
      dataBuffer[dataBufferIndex][1] = event.un.rotationVector.i;
      dataBuffer[dataBufferIndex][2] = event.un.rotationVector.j;
      dataBuffer[dataBufferIndex][3] = event.un.rotationVector.k;

    //printing and clsosing connection if the index is 39
      if(dataBufferIndex == 39){
        //printing to SD card
        for(int i = 0; i<40; i++){
          for(int j = 0; j<numBufferCol; j++){
            dataFile.print(dataBuffer[i][j]);
            dataFile.print(" ");
          }
          dataFile.println();
        }
        //closing connection to sd card
        dataFile.close();
      }
    //increasing index by one, wrapping when needed
      dataBufferIndex = (dataBufferIndex+1) % 40;
  }

//this method will save the data from the flight data buffer array and add it the flight data file. Then, 
//it will close the file
  void driverSD::printFlightDataToFile(File dataFile){
        //printing to SD card
        for(int i = 0; i<=dataBufferIndex; i++){
          for(int j = 0; j<numBufferCol; j++){
            dataFile.print(dataBuffer[i][j]);
            dataFile.print(" ");

          }
          dataFile.println();
        }
        //closing file
        dataFile.close();
      }

//this method will save all of the data in the soil data buffer to the soil data file, then close the file in the case the program shuts down before 40 indexes are reached
  void driverSD::printSoilDataToFile(File dataFile){
        //printing to SD card
        for(int i = 0; i<=dataBufferIndex; i++){
          for(int j = 0; j<numBufferCol; j++){
            dataFile.print(dataBuffer[i][j]);
            dataFile.print(" ");

          }
          dataFile.println();
        }
        //closing file
        dataFile.close();
      }

//This method takes in the flight data buffer index, adds data to array, and returns the index+1 % 40 (will wrap around
//to index zero once it hits 40). If the index is 39, it will print the entire array to the file, then close the file to save.
  void driverSD::addFlightData(sensors_event_t linearAccelData, sh2_SensorValue_t orienData , float32_t altitude, File dataFile){
    //adding data to array
      dataBuffer[dataBufferIndex][0] = (float32_t)millis();
      dataBuffer[dataBufferIndex][1] = linearAccelData.acceleration.x;
      dataBuffer[dataBufferIndex][2] = linearAccelData.acceleration.y;
      dataBuffer[dataBufferIndex][3] = linearAccelData.acceleration.z;
      dataBuffer[dataBufferIndex][4] = orienData.un.rotationVector.i;
      dataBuffer[dataBufferIndex][5] = orienData.un.rotationVector.j;
      dataBuffer[dataBufferIndex][6] = orienData.un.rotationVector.k;
      dataBuffer[dataBufferIndex][7] = altitude;

    //printing data to file and closing connection if index is 39
      if(dataBufferIndex==39){
        printFlightDataToFile(dataFile);
      }
    
    //increasing the index by 1, wrapping if necesarry
      dataBufferIndex = (dataBufferIndex+1) % 40;

  }

//addding soil sensor data
void driverSD::addSoilSensorData(float32_t nitrogenPercentage, float32_t pH, float32_t electricalConductivity, File dataFile){
    //adding data to array
      dataBuffer[dataBufferIndex][0] = (float32_t)millis();
      dataBuffer[dataBufferIndex][1] = nitrogenPercentage;
      dataBuffer[dataBufferIndex][2] = pH;
      dataBuffer[dataBufferIndex][3] = electricalConductivity;

    //printing and clsosing connection if the index is 39
      if(dataBufferIndex == 39){
        printSoilDataToFile(dataFile);
      }
    //increasing index by one, wrapping when needed
      dataBufferIndex = (dataBufferIndex+1) % 40;
  }



