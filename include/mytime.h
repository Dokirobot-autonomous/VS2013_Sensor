#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

/*  ���ԃN���X  */
class MyTime
{
public:
	/*  �R���X�g���N�^  */
	MyTime() :h(0), m(0), s(0){};
	MyTime(int h, int m, int s) :h(h), m(m), s(s){};
	/*  string����  */
	MyTime(const std::string& str)
	{
		std::istringstream istr(str);

		char c;
		istr >> h >> c >> m >> c >> s;
	}

	/*  �f�X�g���N�^  */
	~MyTime(){};

	/*  string�o��  */
	std::string str() const
	{
		char c[128];
		sprintf_s(c, "%02d:%02d:%02d", h, m, s);
		return c;
	}

	/* �t�@�C�����pstring */
	std::string filenameStr() const
	{
		char c[128];
		sprintf_s(c, "%02dh%02dm%02ds", h, m, s);
		return c;
	}

	/*  string����  */
	void str(const std::string& str)
	{
		std::istringstream istr(str);

		char c;
		istr >> h >> c >> m >> c >> s;
	}
	void hoge()
	{
	}
	/*  ��r���Z�q  */
	inline bool operator==(const MyTime& obj)
	{
		return (this->h == obj.h&&this->m == obj.m&&this->s == obj.s) ? true : false;
	}
	inline bool operator!=(const MyTime& obj)
	{
		return (this->h != obj.h || this->m != obj.m || this->s != obj.s) ? true : false;
	}
	inline bool operator>(const MyTime& obj)
	{
		if (this->h > obj.h)		return true;
		else if (this->h < obj.h)	return false;
		else
		{
			if (this->m > obj.m)		return true;
			else if (this->m < obj.m)	return false;
			else
			{
				if (this->s > obj.s)		return true;
				return false;
			}
		}
		std::cout << "Time Error!" << std::endl;
		exit(0);
	}
	inline bool operator<(const MyTime& obj)
	{
		if (this->h < obj.h)		return true;
		else if (this->h > obj.h)	return false;
		else
		{
			if (this->m < obj.m)		return true;
			else if (this->m > obj.m)	return false;
			else
			{
				if (this->s < obj.s)		return true;
				return false;
			}
		}
		std::cout << "Time Error!" << std::endl;
		exit(0);
	}
	inline bool operator>=(const MyTime& obj)
	{
		if (this->h > obj.h)		return true;
		else if (this->h < obj.h)	return false;
		else
		{
			if (this->m > obj.m)		return true;
			else if (this->m < obj.m)	return false;
			else
			{
				if (this->s >= obj.s)		return true;
				return false;
			}
		}
		std::cout << "Time Error!" << std::endl;
		exit(0);
	}
	inline bool operator<=(const MyTime& obj)
	{
		if (this->h < obj.h)		return true;
		else if (this->h > obj.h)	return false;
		else
		{
			if (this->m < obj.m)		return true;
			else if (this->m > obj.m)	return false;
			else
			{
				if (this->s <= obj.s)		return true;
				return false;
			}
		}
		std::cout << "Time Error!" << std::endl;
		exit(0);
	}

	/*  �X�g���[�����Z�q  */
	friend inline std::istream& operator>>(std::istream& is, MyTime& obj);
	friend inline std::ostream& operator<<(std::ostream& os, const MyTime& obj);

	/*  ���ԍ����o��[s](obj2-obj1)  */
	static inline int diff(const MyTime& obj1, const MyTime& obj2)
	{
		int dh = obj2.h - obj1.h;
		int dm = obj2.m - obj1.m;
		int ds = obj2.s - obj1.s;

		return dh * 3600 + dm * 60 + ds;
	}

	/*  �����o�֐�  */
	static void readData(std::istream& is, std::vector<MyTime>& obj, int skip_rows = 0, int skip_cols = 0, char delimiter = ',')
	{
		std::string str;	//	��s���̕�������i�[

		/*  skip_rows�s�X�L�b�v  */
		for (int i = 0; i < skip_rows; i++)
		{
			std::getline(is, str);
		}

		/*  �Ō�̍s�܂œǂݍ���  */
		while (std::getline(is, str))
		{
			std::istringstream istr(str);
			std::string token;

			/*  skip_cols��X�L�b�v*/
			for (int j = 0; j < skip_cols; j++)
			{
				std::getline(istr, token, delimiter);
			}

			/*  �f�[�^�̓ǂݍ���  */
			MyTime tmp;
			istr >> tmp;
			obj.push_back(tmp);
		}

	}

private:
	int h;	//	hour
	int m;	//	minute
	int s;	//	second

};

/*  �X�g���[�����Z�q  */
inline std::istream& operator>>(std::istream& is, MyTime& obj)
{
	char c;
	is >> obj.h >> c >> obj.m >> c >> obj.s;

	return is;
}
inline std::ostream& operator<<(std::ostream& os, const MyTime& obj)
{
	os << obj.str();
	return os;
}
inline std::istream& operator>>(std::istream& is, std::vector<MyTime>& obj)
{
	/*  ������  */
	if (!obj.empty())	obj.clear();

	std::string str;
	while (std::getline(is, str))
	{
		std::istringstream istr(str);
		MyTime tmp;
		istr >> tmp;
		obj.push_back(tmp);
	}
	return is;
}
inline std::ostream& operator<<(std::ostream& os, const std::vector<MyTime>& obj)
{
	for (const auto &col : obj)
	{
		os << col << std::endl;
	}
	return os;
}