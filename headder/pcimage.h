#ifndef _INC_PCIMAGE
#define _INC_PCIMAGE

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>

const int imageNum = 4;		//���O�ɗp�ӂ���摜�̈�̐�

// �f�B���N�g�������w�肵�ă}�b�s���O�p�摜�𐶐�
void uniteImage(std::string dirPath, cv::Point& originXY, cv::Mat& mappImg, int width = 1000);

//�_�Q�摜���쐬����N���X
class PCImage
{
public:
	//Mat�N���X���p�������_�Q�摜�N���X
	//�摜�ʒu���l�������������s��
	class PCI;

	std::string dirname;					//�쐬����f�B���N�g����

	bool isWriteLine = true;					// �}�b�v�Ɏ��Ȉʒu���瑪��_�܂ł̐���`�悷�邩�ǂ���
	static bool isColor;

	enum BGR{B = 1 , G = 2 , R = 4 , GRAY = 8};

private:
	std::vector<PCI> pcimage;				//�摜�̈�̔z��

	int img_width;					//�p�ӂ���摜�̕�
	int img_height;					//�p�ӂ���摜�̍���
	int coefficient;				//�f�[�^���𑜓x�ɍ��킹��W��
	int imgval_increment;			//��f�l�̑�����
	int limit , limitpix;			//���̉摜��ǂݍ��ރ{�[�_�[���C��(m)(pix)

	int origin_x, origin_y;

	int nowimage;							//���ݑ��s���Ă���摜�̔ԍ�
	float selfPos_x, selfPos_y;			// ���Ȉʒu

	const int lineVal = 200;		// ����`�悷��Ƃ��̉�f�l

	cv::Mat arrowpic;

	bool color[3];

/***********************
*	��private���\�b�h��
***********************/
	//���Ȉʒu�ɉ����ĉ摜�̗p�ӂȂǂ̏���������
	int checkPosition(float pos_x, float pos_y);

	// ���Ȉʒu������͈̔͊O�������ꍇ�̏���
	void outsideProcess(int pos_x, int pos_y, int XY[2]);

	//�摜�̗̈�ԍ���₢���킹��Ɛ^�U��Ԃ�
	bool checkPrepare(int x, int y);

	//���̉摜��p�ӂ���
	int prepareImage(int x, int y);

	//�g���Ă��Ȃ��摜�̔ԍ���Ԃ�
	int getEmptyImage();

	//�摜��ǂݍ���
	int loadPCImage(int emptyImageNum);

	//���S�摜���w������ɃV�t�g����
	int shiftCenterImage(int x, int y);

	// ���ݒn�\���p�̖�������
	void prepareArrow();
	// ���ݒn����ŕ\��(������)
	void showArrow();

	void showNowPoint(float x_val , float y_val);

	static cv::Mat initImage(int width,int height);

public:
	//�R���X�g���N�^
	PCImage();
	//�f�X�g���N�^
	~PCImage();

	PCImage& operator=(PCImage& pci);

	// ����������
	void initPCImage();
	void initPCImage(int resolution);
	void initPCImage(int width, int height, int resolution);

	PCImage instantiate();

	// �摜�ɓ_����������
	void writePoint(float x_val, float y_val);
	// �_���������񂾌�ɐ���`�悷��
	void writePoint(float x_val, float y_val , float pos_x , float pos_y);

	// �w�肵����_�����Ԓ�����`��
	void writeLine(float x_val, float y_val, float pos_x, float pos_y);

	// ��f�l���擾����
	int readPoint(int x_val, int y_val);

	// �摜��ۑ�
	//int savePCImage();
	//�摜��ۑ����ė̈���������
	void savePCImage(int x, int y);
	void savePCImage();
	void savePCImage(int num, std::string savename);

	// �ۑ������f�B���N�g�������擾
	std::string getDirname();

	//���݂̎����𕶎���Ŏ擾����
	static void getNowTime(std::string& nowstr);

	void setColor(BGR bgr);

	void setOrigin(int x,int y);
	void getImage(cv::Mat& m , int num = -1);
	
};


//Mat�N���X���p�������_�Q�摜�N���X
//�摜�ʒu���l�������������s��
class PCImage::PCI : public cv::Mat
{
private:
	PCImage& pciOut;	//PCImage�N���X�ւ̎Q��

	std::string			name;					//�ۑ����̖��O
	int					imageNumXY[2];			//�摜�̈ʒu


	void write(int x, int y);

public:
	PCI(PCImage& pcimage_outer);
	PCI& operator=(cv::Mat& mat);

	//�摜�����Z�b�g����
	void setPCI(int x, int y);

	//�摜�̈ʒu��Ԃ�
	void getImageNumber(int xy[]);

	//�摜����Ԃ�
	// [./(directoryname)/(filename).jpg]
	std::string getName();

	//�摜�̗̈�ԍ���₢���킹��Ɛ^�U��Ԃ�
	bool isCoordinates(int x, int y);
	bool isCoordinates(int xy[]);

	//�摜�ɓ_����������
	int writePoint(float x_val, float y_val);

	//�摜��ۑ����ė̈���������
	void release();
	//�摜��ۑ�����
	void savePCImage();
	void savePCImage(std::string savename);

	// ������`�悷��D��f�l��0�łȂ���f�͏㏑�����Ȃ�
	void line(cv::Point start, cv::Point end, int color);

	// �摜���̓_���ǂ������`�F�b�N
	void checkOverRange( int x_coord , int y_coord , int& ret_x , int& ret_y );
};



#endif