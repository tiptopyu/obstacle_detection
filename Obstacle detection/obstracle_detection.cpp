#define _USE_MATH_DEFINES

#include "urg_sensor.h"
#include "urg_utils.h"
#include <stdlib.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int obstacle_detection(urg_t urg)
{
	int URG_L = 4000;	//URG���苗��
	int length_data_size;	//����_�̐�
	long *length_data;		//���苗���̊i�[��
	long min_distance, max_distance;	//����\����

	int **p_data;

	int i, j;
	int p_x, p_y;
	double slope;
	int x_division=20, y_division=20;//x,y�������̕�����
	long Disx_resolution = URG_L / x_division;//x�����̕Ӓ���
	long Disy_resolution = URG_L / y_division;//y�����̕Ӓ���
	int center=y_division ;//�Z���T���݈ʒu
	double border_x = 0, border_y = 0, old_border_x, old_border_y;//���E�����ʗp�ϐ�

	p_data = (int **)malloc(sizeof(int *)*y_division*2);
	for (i = 0; i < y_division*2; i++)
	{
		p_data[i] = (int *)malloc(sizeof(int *)*x_division);
	}
	for (j = 0; j <x_division; j++)
	{
		for (i = 0; i <y_division*2; i++)
		{
			p_data[i][j] = 2;
		}

	}
	length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
	urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
	length_data_size = urg_get_distance(&urg, length_data, NULL);				//���肵���_�Q�̋����擾
	urg_distance_min_max(&urg, &min_distance, &max_distance);					//����\�͈͂̒�`

	for (i = 0; i < length_data_size; ++i) {

		// \~japanese ���̋����f�[�^�̃��W�A���p�x�����߁AX, Y �̍��W�l���v�Z����
		double radian;
		long length;
		int x;
		int y;

		radian = urg_index2rad(&urg, i);
		//printf("%lf\n", radian);
		length = length_data[i];
		// \todo check length is valid

		if ((length > min_distance) && (length < max_distance))
		{
			if (radian<M_PI / 2 && radian>-M_PI / 2)
			{
				p_x = (length * cos(radian)) / Disx_resolution;
				p_y = ((length * sin(radian)) + URG_L) / Disy_resolution;


				p_data[p_y][p_x] = 1;
				old_border_x = p_x;
				old_border_y = p_y;
				slope = (double)(p_x + 0.5 - 0) / (p_y + 0.5 - center);
				if (abs(slope) <= 1)
				{
					if (slope < 0)
					{//y�}�C�i�X�����ɐi�s

						while (old_border_x >= 0)
						{

							//Sleep(100);
							border_y = old_border_x / slope + center;
							for (old_border_y = floor(old_border_y); old_border_y < ceil(border_y); old_border_y++)
							{


								if (old_border_y >= 0 && p_data[(int)old_border_y][(int)old_border_x] == 2)
									p_data[(int)old_border_y][(int)old_border_x] = 0;
							}
							old_border_y = border_y;
							old_border_x--;


						}

					}
					else
					{//y�v���X�����ɐi�s

						while (old_border_x >= 0)
						{

							border_y = old_border_x / slope + center;

							for (old_border_y = floor(old_border_y); old_border_y >= floor(border_y); old_border_y--)
							{

								if (old_border_y <= 39 && p_data[(int)old_border_y][(int)old_border_x] == 2)
									p_data[(int)old_border_y][(int)old_border_x] = 0;
							}
							old_border_y = border_y;
							old_border_x--;


						}
					}

				}
				else
				{//x�v���X�����ɐi�s

					if (slope == 1)
					{

						for (; old_border_x > 0; old_border_x--, old_border_y--)
						{

							if (p_data[(int)old_border_y][(int)old_border_x] == 2)
							{

								p_data[(int)old_border_y][(int)old_border_x] = 0;
							}
						}
					}
					if (slope < 0)
					{

						while (old_border_x >= 0)
						{
							border_x = (old_border_y + 1 - center) * slope + 0;

							for (old_border_x = floor(old_border_x); old_border_x >= floor(border_x); old_border_x--)
							{

								if (old_border_x <= 19 && p_data[(int)old_border_y][(int)old_border_x] == 2)
								{

									p_data[(int)old_border_y][(int)old_border_x] = 0;
								}

							}
							old_border_y++;
							old_border_x = border_x;


						}
					}
					else
					{
						while (old_border_x >= 0)
						{
							border_x = (old_border_y + 1 - center) * slope + 0;

							for (old_border_x = floor(old_border_x); old_border_x >= floor(border_x); old_border_x--)
							{

								if (old_border_x <= 19 && p_data[(int)old_border_y][(int)old_border_x] == 2)
								{

									p_data[(int)old_border_y][(int)old_border_x] = 0;
								}

							}
							old_border_y--;
							old_border_x = border_x;


						}
					}
				}

			}

		}

	}

	int sl_2 = 10;	//�T���̈�̉��s��
	int sr_2 = 5;	//�T���̈�̕�(���E�Б�)

	int re_2 = center - sr_2;	//�T���̈�̉E�[
	int le_2 = center + sr_2;	//�T���̈�̍��[

	bool fr_2 = false;
	for (int i = re_2; i < center; i++){
		for (int j = 0; j < sl_2; j++){
			if (p_data[i][j] != 0)fr_2 = true;
		}
	}

	bool fl_2 = false;
	for (int i = center; i < le_2; i++){
		for (int j = 0; j < sl_2; j++){
			if (p_data[i][j] != 0)fl_2 = true;
		}
	}



	/*��ԓ���*/

	int sl_1 = 5;	//�T���̈�̉��s��
	int sr_1 = 3;	//�T���̈�̕�(���E�Б�)

	int re_1 = center - sr_1;	//�T���̈�̉E�[
	int le_1 = center + sr_1;	//�T���̈�̍��[

	bool fr_1 = false;
	for (int i = re_1; i < center; i++){
		for (int j = 0; j < sl_1; j++){
			if (p_data[i][j] != 0)fr_1 = true;
		}
	}

	bool fl_1 = false;
	for (int i = center; i < le_1; i++){
		for (int j = 0; j < sl_1; j++){
			if (p_data[i][j] != 0)fl_1 = true;
		}
	}

	/*�Ԃ�l*/
	if (fr_2 == false && fl_2 == false)return 0;	//��Q���Ȃ�

	else if (fr_2 == true && fl_2 == false){
		if (fr_1 == false)return 1;					//�E��
		else return 2;								//�E��
	}

	else if (fr_2 == false && fl_2 == true){
		if (fl_1 == false)return 3;					//����
		else return 4;								//����
	}


	else{
		if (fr_1 == false && fl_1 == false)return 5;
		else if (fr_1 == true && fl_1 == false)return 6;
		else if (fr_1 == false && fl_1 == true)return 7;
		else return 8;
	}
}