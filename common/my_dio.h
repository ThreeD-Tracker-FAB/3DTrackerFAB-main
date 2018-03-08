#pragma once

// digital input & output using NI-USB6501 (device name should be "Dev1")

#include <NIDAQmx.h>

#pragma comment(lib, "NIDAQmx.lib")

class MyDIO {
public:


	MyDIO()
	{
		task_handle = 0;

		DAQmxCreateTask("", &task_handle);
		DAQmxCreateDOChan(task_handle, "Dev_3DT/port0/line0:0", "", DAQmx_Val_ChanForAllLines);	// port0.0 for single ch output
		
		DAQmxStartTask(task_handle);

		outTTL(false);
	}

	void outTTL(bool sig)
	{
		uInt8 data = static_cast<uInt8>(sig);

		DAQmxWriteDigitalLines(task_handle, 1, 1, 10.0, DAQmx_Val_GroupByChannel, &data, NULL, NULL);
	}


	~MyDIO()
	{
		DAQmxStopTask(task_handle);
		DAQmxClearTask(task_handle);

	}

private:


	TaskHandle  task_handle;

};

