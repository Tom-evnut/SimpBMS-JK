#include "config.h"
#include "BMSModule.h"
#include "Logger.h"


BMSModule::BMSModule()
{
  for (int i = 0; i < 24; i++)
  {
    cellVolt[i] = 0.0f;
    lowestCellVolt[i] = 5.0f;
    highestCellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  lowestTemperature = 200.0f;
  highestTemperature = -100.0f;
  lowestModuleVolt = 200.0f;
  highestModuleVolt = 0.0f;
  balstat = 0;
  exists = false;
  reset = false;
  moduleAddress = 0;
  timeout = 30000; //milliseconds before comms timeout;
  type = 1;
}

void BMSModule::clearmodule()
{
  for (int i = 0; i < 24; i++)
  {
    cellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  balstat = 0;
  exists = false;
  reset = false;
  moduleAddress = 0;
}

void BMSModule::decodecan(CAN_message_t &msg)
{
  uint8_t Id = 0;

  Id = msg.buf[0];

  switch (Id)
  {
    case 1:
      temperatures[0] = msg.buf[2] + msg.buf[1] * 256;
      break;

    case 2:
      balstat = msg.buf[3];
      break;

    case 4:
      cellVolt[msg.buf[1]] = (msg.buf[3] + msg.buf[2] * 256) * 0.001;
      cellVolt[msg.buf[1] + 1] = (msg.buf[5] + msg.buf[4] * 256) * 0.001;
      cellVolt[msg.buf[1] + 2] = (msg.buf[7] + msg.buf[6] * 256) * 0.001;
      break;

    default:
      break;
  }
}


/*
  Reading the status of the board to identify any flags, will be more useful when implementing a sleep cycle
*/

uint8_t BMSModule::getFaults()
{
  return faults;
}

uint8_t BMSModule::getAlerts()
{
  return alerts;
}

uint8_t BMSModule::getCOVCells()
{
  return COVFaults;
}

uint8_t BMSModule::getCUVCells()
{
  return CUVFaults;
}

float BMSModule::getCellVoltage(int cell)
{
  if (cell < 0 || cell > 24) return 0.0f;
  return cellVolt[cell];
}

float BMSModule::getLowCellV()
{
  float lowVal = 10.0f;
  for (int i = 0; i < 24; i++) if (cellVolt[i] < lowVal && cellVolt[i] > IgnoreCell) lowVal = cellVolt[i];
  return lowVal;
}

float BMSModule::getHighCellV()
{
  float hiVal = 0.0f;
  for (int i = 0; i < 24; i++)
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 5.0)
    {
      if (cellVolt[i] > hiVal) hiVal = cellVolt[i];
    }
  return hiVal;
}

float BMSModule::getAverageV()
{
  int x = 0;
  float avgVal = 0.0f;
  for (int i = 0; i < 24; i++)
  {
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 5.0)
    {
      x++;
      avgVal += cellVolt[i];
    }
  }

  scells = x;
  avgVal /= x;

  if (scells == 0)
  {
    avgVal = 0;
  }

  return avgVal;
}

int BMSModule::getscells()
{
  return scells;
}

float BMSModule::getHighestModuleVolt()
{
  return highestModuleVolt;
}

float BMSModule::getLowestModuleVolt()
{
  return lowestModuleVolt;
}

float BMSModule::getHighestCellVolt(int cell)
{
  if (cell < 0 || cell > 13) return 0.0f;
  return highestCellVolt[cell];
}

float BMSModule::getLowestCellVolt(int cell)
{
  if (cell < 0 || cell > 13) return 0.0f;
  return lowestCellVolt[cell];
}

float BMSModule::getHighestTemp()
{
  return highestTemperature;
}

float BMSModule::getLowestTemp()
{
  return lowestTemperature;
}

float BMSModule::getLowTemp()
{
  return temperatures[0];
}

float BMSModule::getHighTemp()
{
  return temperatures[0];
}

float BMSModule::getAvgTemp()
{
  return temperatures[0];
}

float BMSModule::getModuleVoltage()
{
  moduleVolt = 0;
  for (int I; I < 13; I++)
  {
    if (cellVolt[I] > IgnoreCell && cellVolt[I] < 5.0)
    {
      moduleVolt = moduleVolt + cellVolt[I];
    }
  }
  return moduleVolt;
}

float BMSModule::getTemperature(int temp)
{
  if (temp < 0 || temp > 2) return 0.0f;
  return temperatures[temp];
}

void BMSModule::setAddress(int newAddr)
{
  if (newAddr < 0 || newAddr > MAX_MODULE_ADDR) return;
  moduleAddress = newAddr;
}

int BMSModule::getAddress()
{
  return moduleAddress;
}

int BMSModule::getType()
{
  return type;
}

int BMSModule::getBalStat()
{
  return balstat;
}

bool BMSModule::isExisting()
{
  return exists;
}

bool BMSModule::isReset()
{
  return reset;
}

void BMSModule::settempsensor(int tempsensor)
{
  sensor = tempsensor;
}

void BMSModule::setExists(bool ex)
{
  exists = ex;
}

void BMSModule::setDelta(float ex)
{
  VoltDelta = ex;
}

void BMSModule::setReset(bool ex)
{
  reset = ex;
}

void BMSModule::setIgnoreCell(float Ignore)
{
  IgnoreCell = Ignore;
  Serial.println();
  Serial.println();
  Serial.println(Ignore);
  Serial.println();

}
