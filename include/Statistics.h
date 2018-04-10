#pragma once

#ifndef STATISTICS_H_
#define STATISTICS_H_

#include <vector>
#include <numeric>
#include <algorithm>
#include "myfun.h"
#include "parameter.h"

template<typename T = double>
class Statistics
{
public:

	Statistics(){};

	Statistics(std::vector<T> &in)
	{
		this->in.push_back(&in);
		data_num++;
		data_size = in.size();
	}

	Statistics(std::vector<std::vector<T>*> &in)
	{
		this->in = in;
		data_num = in->size();
		data_size = in->front().size();
	}

	~Statistics()
	{
	}

	/*  �f�[�^�̒ǉ�  */
	void add_data(std::vector<T> &in)
	{
		data.push_back(&in);
		data_num++;
		data_size = in.size();
		if (data_size != 0 && data_size != in.size()){
			std::cerr << "likelihoods data size is not same!" << std::endl;
			exit(0);
		}
	}

	/*  ����  */
	void calcMean()
	{
		if (!mean.empty())
		{
			return;
		}
		mean.reserve(data_num);

		//	calculate
		for (const auto data_tmp : data)
		{
			double sum = std::accumulate(data_tmp->begin(), data_tmp->end(), 0.0);
			sum /= (double)data_size;
			mean.push_back(sum);
		}
	}
	std::vector<double> Mean()
	{
		calcMean();
		return mean;
	}

	/*  �����U�s��  */
	void calcCovariance()
	{
		if (!covariance.empty())
		{
			return;
		}

		calcMean();

		//	���U�����U�s���data_num*data_num�̍s��Ŋm��
		covariance = std::vector<std::vector<double>>(data_num, std::vector<double>(data_num));

		/*	�Z�o  */
		//	[ni,nj]=[0,0]->[1,0]->[1,1]->[2,0]->...�̏��Ɍv�Z
		//	[ni,nj]=[nj,ni]�������Ɏ��s
		for (int ni = 0; ni < data_num; ni++){
			for (int nj = 0; nj < ni + 1; nj++){
				double tmp = 0.0;
				for (int si = 0; si < data_size; si++){
					tmp += (data[ni]->at(si) - mean[ni])*(data[nj]->at(si) - mean[nj]);
				}
				covariance[ni][nj] = tmp / (double)data_size;
				if (ni != nj){
					covariance[nj][ni] = covariance[ni][nj];
				}
			}
		}
	};
	std::vector<std::vector<double>> Covariance()
	{
		calcCovariance();
		return covariance;
	}

	/*  �s�A�\���̐ϗ����֌W��  */
	void calcPearsonCorrcoef()
	{
		if (!pearson_corr.empty())
		{
			return;
		}

		Covariance();

		//	���֌W���s���data_num*data_num�̍s��Ŋm��
		pearson_corr = std::vector<std::vector<double>>(data_num, std::vector<double>(data_num));

		/*	�Z�o  */
		//	[ni,nj]=[0,0]->[1,0]->[1,1]->[2,0]->...�̏��Ɍv�Z
		//	[ni,nj]=[nj,ni]�������Ɏ��s
		for (int ni = 0; ni < data_num; ni++){
			for (int nj = 0; nj < ni + 1; nj++){
				pearson_corr[ni][nj] = covariance[ni][nj] / std::sqrt(covariance[ni][ni] * covariance[nj][nj]);
				if (ni != nj){
					pearson_corr[nj][ni] = pearson_corr[ni][nj];
				}
			}
		}
	}
	std::vector<std::vector<double>> pearsonCorr()
	{
		calcPearsonCorrcoef();
		for (auto& tmp : pearson_corr){
			for (auto& tmp1 : tmp){
				if (std::abs(tmp1) < 0.000001){
					tmp1 = 0;
				}
			}
		}
		return pearson_corr;
	}

	/*  �f�[�^�Ԃ̐�  */
	std::vector<T> Product(std::vector<bool> use_,IntegrateType integrate_type)
	{
		std::vector<T> out;

		//	�g�p����f�[�^���Ȃ��ꍇ�C�S�Ă𓝍�����
		if (std::find(use_.begin(), use_.end(),true) == use_.end())
		{
			for (int i = 0; i < data_num; i++)
			{
				use_[i] = true;
			}
		}

		switch (integrate_type)
		{
		case AND:
			out = std::vector<T>(data_size, 1.0);
			for (int n = 0; n < data_num; n++)
			{
				if (use_[n] == false)
					continue;

				for (int s = 0; s < data_size; s++)
				{
					out[s] *= data[n]->at(s);
				}
			}
			break;
		case OR:
			out = std::vector<T>(data_size, 0.0);
			for (int n = 0; n < data_num; n++)
			{
				if (use_[n] == false)
					continue;

				for (int s = 0; s < data_size; s++)
				{
					out[s] += data[n]->at(s);
				}
			}

			break;
		default:
			std::cout << "integrate_type: " << integrate_type << std::endl;
			exit(0);
			break;
		}


		return out;

	}

	/*  ���ϕ��z  */
	std::vector<T> average()
	{
		std::vector<T> out(data_size, 0.0);

		for (const auto& dtmp : data)
		{
			for (int pi = 0; pi < data_size; pi++)
			{
				out[pi] += dtmp->at(pi);
			}
		}

		Normalize(out);

		return out;
	}

	/*  KL����  */
	std::vector<double> KlDivergence(const std::vector<T>& base)
	{
		std::vector<double> out(data_num, 0.0);

		for (int ni = 0; ni < data_num; ni++)
		{
			for (int si = 0; si < data_size; si++)
			{
				if (data[ni]->at(si) != 0.0&&base[si] != 0.0)
				{
					out[ni] += data[ni]->at(si)*std::log2(data[ni]->at(si) / base[si]);
				}
			}
		}

		return out;
	}

	/*  ��敽�ϕ�����  */
	static inline double rootMeanSquare(std::vector<double> kl)
	{
		assert(!kl.empty());
		double sum = 0.0;
		for (const auto& ktmp : kl)
		{
			sum += ktmp*ktmp;
		}
		return std::sqrt(sum / (double)kl.size());
	}

private:
	/**********************************************************/
	//	�ϐ���`
	/**********************************************************/

	std::vector<std::vector<T>*> data;	//	input data	data[data num][point]
	int data_num = 0;
	int data_size = 0;

	std::vector<double> mean;
	std::vector<std::vector<double>> covariance;
	std::vector<std::vector<double>> pearson_corr;	//	Pearson Corrcoef
};

#endif /* STATISTICS_H_ */