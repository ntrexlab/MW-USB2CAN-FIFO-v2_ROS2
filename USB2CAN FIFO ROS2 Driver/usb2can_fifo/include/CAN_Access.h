#include <time.h>

static int find_empty_can_bus_slot ();
long CAN_OpenFifo(const char *serialNumber);
int CAN_Fifo_ScanSerialNumber ();
char* CAN_Fifo_GetSerialNumber (int index);
int  CAN_SetRxEventNotification (long handle, void *hEvent);
int CAN_SetTransferMode (long handle, int mode);
int CAN_Send (long handle, long ID, int length, char data[8], int Ext, int RTR);
int CAN_SetConfig (long handle, long bitrate, unsigned long filterID, unsigned long filterMask, int startupTransferMode, int busOffRecovery);
int CAN_GetConfig (long handle, long *bitrate, unsigned long *filterID, unsigned long *filterMask, int *startupTransferMode, int *busOffRecovery);
int CAN_CountRxQueue (long handle);
int CAN_Recv (long handle, long *ID, int *length, char data[8], int *Ext, int *RTR);
int CAN_Close(long handle);
