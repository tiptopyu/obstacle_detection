#ifndef _INC_TIMER
#define _INC_TIMER

// 2015/10/05

#include <string>
#include <chrono>
#include <vector>

using namespace std;

class Timer
{
public:
	//�P�ʂ̎��
	static enum tUnit { sec, millisec, microsec, nanosec };
	//�P�ʂ̖��O��Ԃ�
	static string unitname(tUnit unit);

private:
	//�����̂Ō^��Z����`
	typedef std::chrono::time_point<std::chrono::system_clock> _time;

	//�v���J�n�_�C���ԓ_���i�[����
	vector<_time>	rawLap;
	vector<int>	lapTime;

	//�w�肵���P�ʂŎ��Ԃ�Ԃ�
	int getTime(_time start, _time end, tUnit unit);

public:
	//�v���J�n�_
	void Start();

	//�v�����ʕۑ�
	//�����̓t�@�C����,�P��
	//�w�肵���t�@�C�������ɑ��݂�����t�@�C���̍Ō���ɒǋL����
	void Save(string filename, tUnit unit = millisec);

	/*
	 * �w��񐔑O�̃��b�v�^�C������̌o�ߎ��Ԃ��擾
	 * Criteria�F�o�ߎ��Ԃ̊�D
	 *			 getLapTime���Ă񂾎��Ԃ��z��ŕۑ�����Ă���̂ŁC�����O����̌o�ߎ��Ԃ��擾����̂����w�肷��D
	 * unit�F�P�ʂ��w��
	 * isSaveLap�F����ȍ~�̊���ԂƂ��Č��݂�ۑ����邩�w��
	 */
	int getLapTime(int Criteria = 1, tUnit unit = millisec, bool isSavaLap = true);

	// ���ݎ����𕶎���ŕԂ�
	string getNowTime();
};

#endif