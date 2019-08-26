#include "common.hpp"
#include <thread>
#include <mutex>


#define FRAMEBUF_NUM 2
#define FRONTDOOR 0
#define BACKDOOR 1
#define IMAGE_LENGTH 640
#define IMAGE_HEIGHT 480


#define BRIGHTNESS_NUM 3
#define BRIGHTNESS_ADJUSTMENT_NUM 8
#define Manual_Adjustment_MaxNum 2
#define STATISTIC_NUM 5

//const int Global_Adjustment_IRGains[BRIGHTNESS_ADJUSTMENT_NUM] = { 0x15, 0x20, 0x30, 0x40, 0x80, 0xB0, 0xD0, 0xf0 };
//const int Global_Adjustment_IRGains[BRIGHTNESS_ADJUSTMENT_NUM] = { 0x40, 0x55, 0x70, 0x80, 0xA0, 0xB0, 0xD0, 0xF0 };
const int Global_Adjustment_IRGains_Dark[BRIGHTNESS_ADJUSTMENT_NUM] = { 0x20, 0x35, 0x50, 0x70, 0x90, 0xB0, 0xD0, 0xF0 };
const int Global_Adjustment_IRGains_Light[BRIGHTNESS_ADJUSTMENT_NUM] = { 0x40, 0x55, 0x70, 0x80, 0xA0, 0xB0, 0xD0, 0xF0 };

std::mutex mtx_show;


class TYSensor;

struct CallbackData {
	int             index;
	TY_DEV_HANDLE   hDevice;
	bool            exit;
	//DepthRender*    render;
	TYSensor *pThis;
};

class TYSensor{
public:
	TY_INTERFACE_HANDLE hIface;
	TY_DEV_HANDLE hDevice;
	TY_FRAME_DATA frame;
	CallbackData cb_data;
	char uri[32];
	int door_id;
	uint32_t frameSize;
	char* frameBuffer[FRAMEBUF_NUM];


public:
	cv::Mat depth;
	cv::Mat depth2image;
	DepthRender render;


private:
	int current_ir_gain ;
	int last_current_ir_gain ;

	int total_brightness_points ;
	int brightness_areas[BRIGHTNESS_NUM];

	int adjustment_pointer ;
	bool autoDownAdjustLocker ;

	int manualAdjustCounter ;

	int statistic_times ;
	int stat_brightness_areas[BRIGHTNESS_NUM];
	bool brightAdjustLock ;


public:
	TYSensor();
	int DetectDevice(int door_id);
	int OpenDevice();
	void GetDevice();
	int FrameCallback(TY_FRAME_DATA * frame, void * userdata);
	const void *GetData();
	void autoAdjustIRGains();


public:
	~TYSensor();
};




TYSensor::~TYSensor()
{

}

TYSensor::TYSensor(){
	memset(uri, 0, 32);
	hIface = NULL;
	hDevice = NULL;

	current_ir_gain = 0x00;  //当前增益
	last_current_ir_gain = 0x00; //上一次调整增益，防止震荡

	total_brightness_points = 0x00;

	adjustment_pointer = 3;
	autoDownAdjustLocker = false;  //自动调整逐步降低亮度的标志锁

	manualAdjustCounter = Manual_Adjustment_MaxNum;     //手工调节开关计数器

	statistic_times = STATISTIC_NUM;
	for (int i = 0; i<BRIGHTNESS_NUM; i++) {
		brightness_areas[i] = 0x00;  //测试出来的CAM亮度由低到高
		stat_brightness_areas[i] = 0x00;  //连续统计亮度，为了稳定调节
	}
	brightAdjustLock = false; // NOTE: brightAdjustLock
}



void eventCallback(TY_EVENT_INFO *event_info, void *userdata)
{
	if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
		struct CallbackData *pstUsrData = (struct CallbackData *)userdata;
		std::cout << "(" << pstUsrData->pThis->uri << ")" << "=== Event Calllback: Device Offline! doorId=" << pstUsrData->pThis->door_id << std::endl;
	}
}


void TYSensor::GetDevice(){
	std::cout << door_id << ": " << uri << std::endl;
	return;
}



int TYSensor::DetectDevice(int doorid){
	int err;
	door_id = doorid;
	std::string ID, IP;
	std::vector<TY_DEVICE_BASE_INFO> selected;
	err = selectDevice(TY_INTERFACE_ALL, ID, IP, 2, selected);
	/*
	if (err != TY_STATUS_OK || selected.size() != 2){
		return -1;
	}
	*/
	TY_DEVICE_BASE_INFO& selectedDev = selected[door_id];
	memset(uri, 0, 32);
	strncpy(uri, selectedDev.id, 29);
	return 0;
}

int TYSensor::OpenDevice(){
	int err = TY_STATUS_OK;
	int32_t  componentIDs = 0;
	int  Width = IMAGE_LENGTH;
	int  Height = IMAGE_HEIGHT;
	ASSERT_OK(TYUpdateInterfaceList());
	ASSERT_OK(TYOpenInterface("usb-dpb", &hIface));
	ASSERT_OK(TYUpdateDeviceList(hIface));
	ASSERT_OK(TYOpenDevice(hIface, uri, &hDevice));
	err = TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_KEEP_ALIVE_ONOFF, false);
	std::cout << "TYSetEnum=" << err << std::endl;
	componentIDs = TY_COMPONENT_DEPTH_CAM;
	std::cout << "(" << uri << ")" << "=== Configure components, open depth cam==" << componentIDs << std::endl;
	ASSERT_OK(TYEnableComponents(hDevice, componentIDs));

	componentIDs = TY_COMPONENT_BRIGHT_HISTO;
	std::cout << "(" << uri << ")" << "=== Configure components, open bright histo==" << componentIDs << std::endl;
	ASSERT_OK(TYEnableComponents(hDevice, componentIDs));
	if (TYSetInt(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_GAIN, Global_Adjustment_IRGains_Dark[adjustment_pointer]) == TY_STATUS_OK
		&& TYSetInt(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_GAIN, Global_Adjustment_IRGains_Dark[adjustment_pointer]) == TY_STATUS_OK) {
		std::cout << "****************=== SET TY_COMPONENT_IR_CAM = " << current_ir_gain << std::endl;
	}

	std::cout << "(" << uri << ")" << "=== Configure feature, set resolution to 640x480." << componentIDs << std::endl;
	err = TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_DEPTH16_640x480);
	ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);
	frameSize = 0;
	ASSERT_OK(TYGetFrameBufferSize(hDevice, &frameSize));
	std::cout << "(" << uri << ")" << "Get size of framebuffer" << " frameSize " << componentIDs << std::endl;
	ASSERT(frameSize >= Width * Height * 2);
	std::cout << "(" << uri << ")" << "     - Allocate & enqueue buffers " << componentIDs << std::endl;

	for (int i = 0; i<FRAMEBUF_NUM; i++)
	{
		frameBuffer[i] = new char[frameSize];
		ASSERT_OK(TYEnqueueBuffer(hDevice, frameBuffer[i], frameSize));
	}

	cb_data.index = 0;
	cb_data.hDevice = hDevice;
	cb_data.pThis = this;
	
	ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, &cb_data));

	ASSERT_OK(TYStartCapture(hDevice));
	std::cout << "(" << uri << ")" << "Get Ready" << std::endl;

	cb_data.exit = false;

	return err;
}


int TYSensor::FrameCallback(TY_FRAME_DATA* frame, void* userdata){
	CallbackData* pData = (CallbackData*)userdata;


	for (int i = 0; i < frame->validCount; i++)
	{
		// get left ir image
		if (frame->image[i].componentID == TY_COMPONENT_IR_CAM_LEFT)
		{
			//	printf("TY_COMPONENT_IR_CAM_LEFT %d.height  %d width %d --%d \n",i,
			//	frame->image[i].height,frame->image[i].width,frame->image[i].size);
			//m_RecMgr->AddFrame(frame->image[i].buffer, DEPTH);
		}
		// get right ir image
		else if (frame->image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT)
		{
			//	printf("TY_COMPONENT_IR_CAM_RIGHT %d.height %d  width %d-%d \n", i,frame->image[i].height,frame->image[i].width,frame->image[i].size);
			//m_RecMgr->AddFrame(frame->image[i].buffer, DEPTH);
		}
		else if (frame->image[i].componentID == TY_COMPONENT_DEPTH_CAM)
		{
			//printf("TY_COMPONENT_DEPTH_CAM %d.height  %d width %d \n",i, frame->image[i].height,frame->image[i].width);
			//std::cout<< "("<<uri<<")"<<"TY_COMPONENT_DEPTH_CAM "<<i<<".height  "<<frame->image[i].height<<" width "<<frame->image[i].width<<" 1";
			//m_RecMgr->AddFrame(frame->image[i].buffer, DEPTH);
			//std::cout<< "("<<uri<<")"<<"TY_COMPONENT_DEPTH_CAM "<<i<<".height  "<<frame->image[i].height<<" width "<<frame->image[i].width<<" 2";
		}
		//get histo to adjust gains automatically
		else if (frame->image[i].componentID == TY_COMPONENT_BRIGHT_HISTO) {
			// printf("TY_COMPONENT_BRIGHT_HISTO %d.height %d width %d \n",i, frame->image[i].height,frame->image[i].width);
			int32_t *ir_left_his, *ir_right_his;
			ir_left_his = (int32_t *)frame->image[i].buffer;
			ir_right_his = (int32_t *)frame->image[i].buffer + 256;

			memset(&brightness_areas, 0x00, BRIGHTNESS_NUM*sizeof(int));
			int i;
			for (i = 0; i<256; i++) {
				if (i < 21)
					brightness_areas[0] += ir_left_his[i];
				else if (i < 253)
					brightness_areas[1] += ir_left_his[i];
				else
					brightness_areas[2] += ir_left_his[i];
			}

			total_brightness_points = (brightness_areas[0] + brightness_areas[1] + brightness_areas[2]);
			brightness_areas[0] = 100 * (float)brightness_areas[0] / (float)total_brightness_points;
			brightness_areas[1] = 100 * (float)brightness_areas[1] / (float)total_brightness_points;
			brightness_areas[2] = 100 * (float)brightness_areas[2] / (float)total_brightness_points;

			//printf("area[0] = %u, area[1] = %u, area[2] = %u\n", brightness_areas[0], brightness_areas[1], brightness_areas[2]);

			if (brightAdjustLock == false) {
				if (statistic_times > 0) //统计一次
				{
					statistic_times--;
					stat_brightness_areas[0] += brightness_areas[0];
					stat_brightness_areas[1] += brightness_areas[1];
					stat_brightness_areas[2] += brightness_areas[2];
				}
				else {
					stat_brightness_areas[0] = 0.5 + stat_brightness_areas[0] / STATISTIC_NUM;
					stat_brightness_areas[1] = 0.5 + stat_brightness_areas[1] / STATISTIC_NUM;
					stat_brightness_areas[2] = 0.5 + stat_brightness_areas[2] / STATISTIC_NUM;
					//printf("***stat_area[0] = %u, stat_area[1] = %u, stat_area[2] = %u, current_ir_gain = %u\n",	stat_brightness_areas[0], stat_brightness_areas[1], stat_brightness_areas[2], current_ir_gain);
					brightAdjustLock = true;
				}
			}

			//自动调整增益，测试时放在这里，正式程序建议放到外面的while大循环里
			autoAdjustIRGains();
		}
	}
	//printf("=== Callback: Re-enqueue buffer(%p, %p, %d)\n", pData->hDevice, frame->userBuffer, frame->bufferSize);
	//std::cout << "(" << uri << ")" << "=== Callback: Re-enqueue buffer(%p, %d)" << frame->userBuffer << "," << frame->bufferSize << std::endl;
	mtx_show.lock();
	TYEnqueueBuffer(pData->hDevice, frame->userBuffer, frame->bufferSize);
	mtx_show.unlock();
	//std::cout << "(" << uri << ")" << "=== Callback: Re-enqueue buffer(%p, %d) end" << frame->userBuffer << "," << frame->bufferSize << std::endl;
	return 0;
}


void TYSensor::autoAdjustIRGains()
{
	//auto adjust ir gains
	if (brightAdjustLock) {
		//测试代码，如果镜头过曝，逐渐调整亮度
		if (stat_brightness_areas[BRIGHTNESS_NUM - 1] > 10) {// || //最亮区域过大
			//stat_brightness_areas[0] < 5 || //最暗区域过小
			//(stat_brightness_areas[BRIGHTNESS_NUM - 1] > 5 && stat_brightness_areas[BRIGHTNESS_NUM - 2] < 40)) { //最亮区域过于集中，周围都比较暗
			if (adjustment_pointer > 0) {  //防止极端跳变,本行限定了只能降低亮度，不能升高亮度
				adjustment_pointer--;
				adjustment_pointer %= BRIGHTNESS_ADJUSTMENT_NUM; //调整亮度逐渐减小，直到最小
				autoDownAdjustLocker = true;  //自动下降调节标志置位，防止反复调上调下
			}
		}
		else if (stat_brightness_areas[0] > 95 || stat_brightness_areas[BRIGHTNESS_NUM - 1] < 1) { //如果画面太暗 0xf0
			if (autoDownAdjustLocker == false && manualAdjustCounter>0) { //如果当前增益值不是自动调下来的，则可以升高增益
				adjustment_pointer = BRIGHTNESS_ADJUSTMENT_NUM - 1; //直接调整亮度到最亮
				manualAdjustCounter--; //手工经验调节一次，计数器减1
			}
		}
		else if (stat_brightness_areas[0] > 90 || stat_brightness_areas[BRIGHTNESS_NUM - 1] < 1) { //如果画面太暗 0xD0
			if (autoDownAdjustLocker == false && manualAdjustCounter>0) { //如果当前增益值不是自动调下来的，则可以升高增益
				adjustment_pointer = BRIGHTNESS_ADJUSTMENT_NUM - 2; //直接调整亮度到次最亮
				manualAdjustCounter--; //手工经验调节一次，计数器减1
			}
		}
		else if (stat_brightness_areas[0] > 80 || stat_brightness_areas[BRIGHTNESS_NUM - 1] < 1) { //如果画面比较暗 0xB0
			if (autoDownAdjustLocker == false && manualAdjustCounter>0) { //如果当前增益值不是自动调下来的，则可以升高增益
				adjustment_pointer = BRIGHTNESS_ADJUSTMENT_NUM - 3; //直接调整亮度到次亮
				manualAdjustCounter--; //手工经验调节一次，计数器减1
			}
		}
		else if (stat_brightness_areas[0] > 70 || stat_brightness_areas[BRIGHTNESS_NUM - 1] < 1) { //如果画面次次暗 0x80
			if (autoDownAdjustLocker == false && manualAdjustCounter>0) { //如果当前增益值不是自动调下来的，则可以升高增益
				adjustment_pointer = BRIGHTNESS_ADJUSTMENT_NUM - 4; //直接调整亮度到次次亮
				manualAdjustCounter--; //手工经验调节一次，计数器减1
			}
		}
		else if (stat_brightness_areas[0] > 60 || stat_brightness_areas[BRIGHTNESS_NUM - 1] < 1) { //如果画面次次暗 0x40
			if (autoDownAdjustLocker == false && manualAdjustCounter>0) { //如果当前增益值不是自动调下来的，则可以升高增益
				adjustment_pointer = BRIGHTNESS_ADJUSTMENT_NUM - 5; //直接调整亮度到稍亮
				manualAdjustCounter--; //手工经验调节一次，计数器减1
			}
		}

		mtx_show.lock();

		//修改增益
		if (door_id == FRONTDOOR) { //前门
			if (current_ir_gain != Global_Adjustment_IRGains_Dark[adjustment_pointer] && last_current_ir_gain != Global_Adjustment_IRGains_Dark[adjustment_pointer]) {  //防止来回切换亮度，没有必要
				if (TYSetInt(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_GAIN, Global_Adjustment_IRGains_Dark[adjustment_pointer]) == TY_STATUS_OK
					&& TYSetInt(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_GAIN, Global_Adjustment_IRGains_Dark[adjustment_pointer]) == TY_STATUS_OK) {
					last_current_ir_gain = current_ir_gain;
					current_ir_gain = Global_Adjustment_IRGains_Dark[adjustment_pointer];
					//std::cout << "****************=== SET TY_COMPONENT_IR_CAM = " << current_ir_gain << std::endl;
				}
				else  {
					//std::cout << "****************=== SET TY_COMPONENT_IR_CAM failed" << std::endl;
				}
			}

		}
		else { //后门
			if (current_ir_gain != Global_Adjustment_IRGains_Light[adjustment_pointer] && last_current_ir_gain != Global_Adjustment_IRGains_Light[adjustment_pointer]) {  //防止来回切换亮度，没有必要
				if (TYSetInt(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_GAIN, Global_Adjustment_IRGains_Light[adjustment_pointer]) == TY_STATUS_OK
					&& TYSetInt(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_GAIN, Global_Adjustment_IRGains_Light[adjustment_pointer]) == TY_STATUS_OK) {
					last_current_ir_gain = current_ir_gain;
					current_ir_gain = Global_Adjustment_IRGains_Light[adjustment_pointer];
					//std::cout << "****************=== SET TY_COMPONENT_IR_CAM = " << current_ir_gain << std::endl;
				}
				else  {
					//std::cout << "****************=== SET TY_COMPONENT_IR_CAM failed" << std::endl;
				}
			}
		}

		mtx_show.unlock();

		statistic_times = STATISTIC_NUM;
		memset(&stat_brightness_areas, 0x00, BRIGHTNESS_NUM*sizeof(int));
		brightAdjustLock = false;
	}
}


const void *TYSensor::GetData(){
	DepthRender render;

	TY_FRAME_DATA *pFrame = &frame;

	int err = TYFetchFrame(hDevice, &frame, -1);
	if (err != TY_STATUS_OK)
	{
		printf("Drop one frame\n");
	}
	else
	{
		//printf("=== Get frame %d\n", pFrame->validCount);
		for (int i = 0; i < pFrame->validCount; i++)
		{
			// get depth image
			//printf("=========index[%d]==componentid0x%x\n",i,pFrame->image[i].componentID);
			if (pFrame->image[i].componentID == TY_COMPONENT_DEPTH_CAM)
			{
				//printf("=== Callback: Re-enqueue buffer(%p, %d)\n", pFrame->userBuffer, pFrame->bufferSize);
				ASSERT_OK(TYEnqueueBuffer(hDevice, pFrame->userBuffer, pFrame->bufferSize));
				this->depth = cv::Mat(frame.image[i].height, frame.image[i].width
					, CV_16U, frame.image[i].buffer).clone();
				this->depth2image = this->render.Compute(this->depth);
				return pFrame->image[i].buffer;
			}
		}
	}
	return NULL;
}


