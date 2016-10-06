#ifndef _INC_RCVDROIDSENSORS
#define _INC_RCVDROIDSENSORS

// 2015/10/09

#include <fstream>
#include <Windows.h>
#include <Timer.h>
#include <SharedMemory.h>

class rcvAndroidSensors
{
private:
	// �����o�ϐ�
	// �V���A���ʐM�n
	int		COM;	//COM�|�[�g�ԍ�
	HANDLE	hComm;	//�V���A���|�[�g�̃n���h��

	// GPS�n
	bool	isSaveGPSCSV = false;
	bool	isPreparationGPSofs = false;
	float	mLatitude , mLongitude , mAccuracy;	//	GPS����擾����ܓx�o�x���x
	Timer	timerGPS;
	ofstream	ofsGPS;
	int		timeCountGPS;

	// ���ʊp�n
	bool	isSaveOrientationCSV  = false;
	bool	isPreparationOrientationofs = false;
	float	mAzimuth, mPitch, mRoll;	// �p��
	Timer	timerOrientation;
	ofstream	ofsOrientation;
	int		timeCountOrientation;

	// ���L�������n
	bool	isSaveSharedMemory;
	const string shMemName = "AndroidSensors";
	SharedMemory<float> shMem;
	enum {ISSAVE , LATITUDE , LONGITUDE , ACCURACY , AZIMUTH , PITCH , ROLL};

	// ���̑�
	int		minSaveInterval;

	// private���\�b�h
	// �V���A���|�[�g���J��
	void	comOpen();
	// �|�[�g�����
	void	comClose();

	void	PreparationGPSCSV();
	void	PreparationOrientationCSV();

public:
	bool	isGetGPS = false;
	bool	isGetOrientation = false;

	//public���\�b�h
	rcvAndroidSensors();
	rcvAndroidSensors( int comport );
	~rcvAndroidSensors();

	void setAndroidSensors(int comport);

	// �f�[�^�̍X�V
	void	getSensorData();

	// �f�[�^���擾���邩�ݒ�
	void	setIsGetGPS(bool isGetdata);
	void	setIsSaveGPSCSV(bool isSaveCSV);
	void	setIsGetOrientation(bool isGetdata);
	void	setIsSaveOrientationCSV(bool isSaveCSV);
	void	setIsSaveSharedMemory(bool isSaveSharedMemory);

	// �ۑ��̍Œ�Ԋu[msec]
	void	setSaveMinInterval(int interval);

	// �f�[�^���擾
	void	getGPSData(float retArray[3]);
	void	getOrientationData(float retArray[3]);
	
};

#endif