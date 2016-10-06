#ifndef _INC_URG_UNKO
#define _INC_URG_UNKO

#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_open.h"
#include "pcimage.h"

#include <Windows.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>

using namespace std;

#define DEBUG_WRITELINE


//�ڑ�����URG�̌��������Ŕ��f����悤�ɂ����}�N��
#define getDataUNKO(aURGCOM , aURGPOS , ARDUINOCOM) getDataUNKOOrigin( (aURGCOM),(aURGPOS),(ARDUINOCOM),sizeof((aURGCOM))/sizeof(aURGCOM[0])) 

//�w�肵��COM�|�[�g�����
int CommClose(HANDLE hComm);
//Arduino�̃n���h�����擾
void getArduinoHandle(int arduinoCOM, HANDLE& hComm);
//urg_unko��main���[�v
void getDataUNKOOrigin(int URG_COM[], float URGPOS[][4], int ARDUINO_COM, int NumOfURG);

class writePCD:ofstream
{
private:
	//std::ofstream ofs;	//�t�@�C���X�g���[���I�u�W�F�N�g�Dpcd�t�@�C���쐬�ɗp����
	static int pcdnum;			//pcd�t�@�C���̔ԍ����J�E���g����ϐ�
	int pcdcount;		//pcd�t�@�C���ɏ������ޓ_�̐����J�E���g����ϐ�

	std::string dirname;

public:
	bool isWritePCD;

	writePCD(std::string dirName = "");
	~writePCD();
	//pcd�t�@�C�����쐬���ď������ޏ������s��
	void pcdinit();
	void pcdinit3D();
	//pcd�t�@�C���֓_����������
	void pcdWrite(float x, float y);
	void pcdWrite(float x, float y, float z);
	void pcdWrite(float x, float y, float pos_x, float pos_y);
	void pcdWrite(float x, float y, float pos_x, float pos_y, float dist, float rad);
	void pcdWrite3D(float x, float y, float pos_x, float pos_y, float dist, float rad);
	void pcdWrite(float x, float y, float pos_x, float pos_y, float droidAngle[], float droidGPS[]);
	//pcd�t�@�C���ւ̏������݂��I�����ĕۑ�����
	void pcdSave();
	void setDirName(std::string dirname);
};


/*
*
*�@URG�Ńf�[�^���擾���ă}�b�s���O���s���N���X
*
*/
class urg_unko
{
protected:
	/*
	*	�����o�ϐ�
	*/
	int COMport;	//URG��COM�|�[�g

	int COMport3D;

	//float urgpos[3];	
	//NCWC�̉�]���S���猩��URG�̈ʒu�D�Z���T�̒n�ʂ���̍����C�Z���T�̊�ʒu����̋����C����ѐ����ʂ���̘�p

	float urgpos[4];	
	//NCWC�̉�]���S���猩��URG�̈ʒu�D�Z���T�̒n�ʂ���̍����C�Z���T�̊�ʒu�����x�����̋����C�������̋����C����ѐ����ʂ���̘�p
	
	float urgpos3D[4];

	urg_t urg;			//URG�I�u�W�F�N�g
	long *data = NULL;	
	long time_stamp;

	float scaninterval = 0.0;//�v�������{����Œ�Ԋu[mm]

	enum {
		CAPTURE_TIMES = 1,
	};

	float currentCoord_x = 0.0, currentCoord_y = 0.0;//�Ԉ֎q�̍��W
	float distance = 0.0, distance_old = 0.0;//�Ԉ֎q�̈ړ�����
	float rob_x= 0.0, rob_y= 0.0;//spur�̍��W
	float radian = 0.0;

	float* pointpos[2];//URG����_
	float* rawpointpos[2];
	int data_n;

	static writePCD pcd;

	/***********************
	 *	private�ȃ��\�b�h  *
	 ***********************/

	//URG�Ƃ̐ڑ���ؒf
	int disconnectURG();
	//URG�Ɛڑ�
	int connectURG();

	//�擾�����f�[�^������ۂ̓񎟌������v�Z���ă}�b�v�Cpcd�t�@�C���ւ̏������݂��s��
	void calcSurface2D();

	//�擾�����f�[�^������ۂ̎O���������v�Z���ĎO�����}�b�v�Cpcd�t�@�C���ւ̏������݂��s��
	void calcSurface3D();

public:
	/*
	*	public�ȃ��\�b�h
	*/
	//�R���X�g���N�^
	urg_unko();
	//�f�X�g���N�^
	virtual ~urg_unko();

	//���g�̏������������s��
	void init(int COM, float pos[]);
	void init3D(int COM, float pos[]);
	//URG����f�[�^���擾���郁�\�b�h
	int getData4URG(float dist, float old, float rad, float spur_x, float spur_y);

	void savePCD();
	void saveRawPCD(float dist,float rad);
	void saveRawPCD3D(float dist, float rad);

	void updateCurrentCoord(float coord_x, float coord_y);
	void updateCurrentCoord(float coordXY[]);

	int getData_n();
	void getData(float data[], int data_n, int offset = 0);
};

class urg_mapping : 
	public urg_unko
{
private:
	static PCImage pcimage;	//�}�b�v�摜�쐬�p�N���X

public:
	~urg_mapping();

	void setWriteLine(bool isLine);
	std::string	getDirName();
	void setPCDDir(std::string dirname = "");

	static void initPCImage(PCImage& pci);
	static void initPCImage(int width, int height, int resolution);
	static void setPCImageOrigin(int x, int y);
	static void getPCImage(cv::Mat& m, int num = -1);

	void setPCImageColor(PCImage::BGR bgr);

	void writeMap(float dist, float old, float rad, float spur_x, float spur_y);
};

#endif