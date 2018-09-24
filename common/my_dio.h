#pragma once

// digital input & output using NI-USB6501 (device name should be "Dev1")

#include <NIDAQmx.h>

#pragma comment(lib, "NIDAQmx.lib")

class MyDIO {
public:


	MyDIO()
	{
		// output
		task_handle_out = 0;

		DAQmxCreateTask("my_output", &task_handle_out);
		DAQmxCreateDOChan(task_handle_out, "Dev_3DT/port0/line0:0", "", DAQmx_Val_ChanForAllLines);	// port0.0 for single ch output
		
		DAQmxStartTask(task_handle_out);

		// input
		task_handle_in = 0;
		DAQmxCreateTask("my_input", &task_handle_in);
		DAQmxCreateDIChan(task_handle_in, "Dev_3DT/port1/line0:1", "", DAQmx_Val_ChanForAllLines);
		DAQmxStartTask(task_handle_in);

		outTTL(false);
	}

	void outTTL(bool sig)
	{
		uInt8 data = static_cast<uInt8>(sig);

		DAQmxWriteDigitalLines(task_handle_out, 1, 1, 10.0, DAQmx_Val_GroupByChannel, &data, NULL, NULL);
	}

	bool readInput(int ch)
	{
		int32	read, bytesPerSamp;
		uInt8	data[2];

		DAQmxReadDigitalLines(task_handle_in, 1, 10.0, DAQmx_Val_GroupByChannel, data, 2, &read, &bytesPerSamp, NULL);

		return (data[ch] != 0);
	}


	~MyDIO()
	{
		DAQmxStopTask(task_handle_out);
		DAQmxClearTask(task_handle_out);

		DAQmxStopTask(task_handle_in);
		DAQmxClearTask(task_handle_in);
	}

private:


	TaskHandle  task_handle_out;
	TaskHandle  task_handle_in;


};

