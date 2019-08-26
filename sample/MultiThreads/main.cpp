#include "../common/TYSensor.cpp"




void *FrameDataThread(void *userdata)
{
	struct CallbackData *pstUsrData = (struct CallbackData *)userdata;
	mtx_show.lock();
	std::cout << "(" << pstUsrData->pThis->uri << ")" << "FrameDataThread start.. " << std::endl;
	mtx_show.unlock();
	TY_FRAME_DATA frame;
	int index = 0;
	int err = 0;
	while (!pstUsrData->exit) {

		
		mtx_show.lock();
		err = TYFetchFrame(pstUsrData->pThis->hDevice, &frame, 200);
		mtx_show.unlock();

		if (err == TY_STATUS_OK) {
			//LOG(INFO)<<"Get frame "<<++index;
			pstUsrData->pThis->FrameCallback(&frame, userdata);
			pstUsrData->pThis->GetData();
			mtx_show.lock();
			imshow(std::to_string(pstUsrData->pThis->door_id), pstUsrData->pThis->depth2image);
			cvWaitKey(1);
			mtx_show.unlock();
		}
		//usleep(2000);
	}

	mtx_show.lock();
	std::cout << "(" << pstUsrData->pThis->uri << ")" << "FrameDataThread exit.. " << std::endl;
	mtx_show.unlock();
	return 0;
}



int main(int argc, char* argv[])
{
	TYSensor sensor1, sensor2;
	int err1, err2;
	ASSERT_OK(TYInitLib());

	err1 = sensor1.DetectDevice(FRONTDOOR);
	//err2 = sensor2.DetectDevice(BACKDOOR);
	/*
	while (err1 != TY_STATUS_OK || err2 != TY_STATUS_OK){
		std::cout << "=== CHECKING CAMERA ..." << std::endl;
		err1 = sensor1.DetectDevice(FRONTDOOR);
		err2 = sensor2.DetectDevice(BACKDOOR);
	}
	*/

	sensor1.OpenDevice();
	//sensor2.OpenDevice();

	std::thread *thread_1 = new std::thread(FrameDataThread, &sensor1.cb_data);
	//std::thread *thread_2 = new std::thread(FrameDataThread, &sensor2.cb_data);

	thread_1->join();
	//thread_2->join();

	delete thread_1;
	//delete thread_2;
	
	ASSERT_OK(TYDeinitLib());
	system("pause");
	return 0;
}
