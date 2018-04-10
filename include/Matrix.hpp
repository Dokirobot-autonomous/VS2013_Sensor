#pragma once

template<typename T> class DMatrix{
public:
	class iterator{
		T* iptr;
	public:
		iterator():iptr(NULL){}
		iterator(const iterator& obj):iptr(obj.iptr){}
		iterator(T* obj):iptr(obj){}
		iterator& operator=(T* obj){
			iptr = obj;
			return *this;
		}
		iterator& operator=(const iterator& obj){
			iptr = obj.iptr;
			return *this;
		}
		bool operator!=(const iterator& obj){
			return (iptr != obj.iptr);
		}
		iterator& operator++(){
			++iptr;
			return *this;
		}
		T& operator*(){ return *iptr; }
		T* operator->(){ return iptr; }
	};
private:
	T** ptr;
	int n_min, n_max, m_min, m_max;
	int n_, m_;
	iterator be,en;
	DMatrix& operator=(const DMatrix&);
	DMatrix(const DMatrix&);
public:
	DMatrix():ptr(NULL){}
	DMatrix(int n1, int n2, int m1, int m2)
		:n_min(std::min(n1,n2)),n_max(std::max(n1,n2)),m_min(std::min(m1,m2)),m_max(std::max(m1,m2))
		,n_(n_max-n_min+1),m_(m_max-m_min+1)
	{
		ptr = Make(n1,n2,m1,m2);
		be = &ptr[n_min][m_min];
		en = (&ptr[n_max][m_max])+1;
	}
	~DMatrix(){
		Delete(ptr,n_min,n_max,m_min,m_max);
	}

	T& operator()(int n, int m){
		return ptr[n][m];
	}

	iterator begin()const{
		return be;
	}
	iterator end()const{
		return en;
	}
public:
	static T** Make(int n1, int n2, int m1, int m2){
		int n_min(std::min(n1,n2)),n_max(std::max(n1,n2)),m_min(std::min(m1,m2)),m_max(std::max(m1,m2));
		int n(n_max-n_min+1), m(m_max-m_min+1);
		T** mat = new T*[n];
		mat[0] = new T[n*m];
		for(int i(1); i<n; ++i) mat[i] = (mat[0] + i*m -m_min);
		mat[0] -= m_min;
		return (mat-n_min);
	}
	static void Delete(T** mat, int n1, int n2, int m1, int m2){
		int n_min(std::min(n1,n2)),m_min(std::min(m1,m2));
		mat += n_min;
		if(mat!=NULL) delete[] (mat[0] + m_min);
		delete[] mat;
		mat = NULL;
	}
};

/*
//Žg—p—á
void example1(){
	int x1(-1),x2(-3),y1(-1),y2(-2);
	DMatrix<int> dm(y1,y2,x1,x2);
	int i(0);
	for(int y(min(y1,y2)); y<=max(y1,y2); ++y){
		for(int x(min(x1,x2)); x<=max(x1,x2); ++x){
			dm(y,x) = i;
			std::cout<< i <<std::endl;
			++i;
		}
	}
	std::cout<<std::endl;
	for(int y(min(y1,y2)); y<=max(y1,y2); ++y){
		for(int x(min(x1,x2)); x<=max(x1,x2); ++x){
			std::cout<< dm(y,x) <<" ";
		}
		std::cout<<std::endl;
	}
	std::cout<<std::endl;
	i=0;
	for(DMatrix<int>::iterator it(dm.begin()); it!=dm.end(); ++it){
		std::cout<< *it;
		std::cout<< ((++i%(abs(x1-x2)+1))!=0)?(" "):("\n");
	}
}
void example2(){
	int x1(-1),x2(-3),y1(-1),y2(-2);
	int** dm = DMatrix<int>::Make(x1,x2,y1,y2);
	int i(0);
	for(int y(min(y1,y2)); y<=max(y1,y2); ++y){
		for(int x(min(x1,x2)); x<=max(x1,x2); ++x){
			dm[x][y] = i;
			std::cout<< i <<std::endl;
			++i;
		}
	}
	std::cout<<std::endl;
	for(int y(min(y1,y2)); y<=max(y1,y2); ++y){
		for(int x(min(x1,x2)); x<=max(x1,x2); ++x){
			std::cout<< dm[x][y] <<" ";
		}
		std::cout<<std::endl;
	}
	DMatrix<int>::Delete(dm,x1,x2,y1,y2);
}
*/
