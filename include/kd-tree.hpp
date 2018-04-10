//---�ċߖT�_�T���N���XVer.4.1---//
//����:
//      k-D tree �ɂ�����
//      �Ώۂ�x-y�̒������W�n��̓_ (2����)
//���e:
//      makeTree �����s�����k-D�؂������ (�����̖؂ɐV���Ƀf�[�^��ǉ�����@�\�͖��� �����̂Ƃ����蒼���őΉ�)
//      searchNeighborPoint �ŋߖT�_��Ԃ�
//      findPoint �؂̒��Ɏw�肵���f�[�^������ꍇ true ��Ԃ�
//      getDistance �ŋߖT�_�Ƃ̋�����Ԃ�
//�O�񂩂�̕ύX:
//      Ver.4.0: �����̒ǉ�, ������
//      Ver.4.1: �ċߖT�_�Ƃ��̎��ɋ߂��_��Ԃ��悤�ɂ����֐���ǉ� (2012:11:27)


#pragma once
#include "Coor.hpp"
#include <vector>
#include <algorithm>

class KDtree{
private:
	struct Node{
		Coor<> point;
		double data;
		Node *right,*left;
		Node():point(0,0),data(0),right(NULL),left(NULL){}
		Node& operator=(const Coor<> &rhs){point=rhs;return *this;}
	};
	std::vector<Node*> nodeList;//delete�p
	Node* root;//��
	std::vector< Coor<> > object;//�؂ɂ���Ώ�
	Coor<> referenceData;//�ŋߖT�_�T�����̊
	Coor<> nearestNeighbor;//�ŋߖT�_
	double distance;//���� �ŋߖT�_�T����͍ŏ�����
	Coor<> circleMin,circleMax;//�ŋߖT�_�T�����̊�𒆐S�Ƃ���~�͈̔�

	//�ߖT��_�����
	std::pair<Coor<>,Coor<> > nearestNeighbor2;//�ŋߖT�_
	std::pair<double,double> distance2;//���� �ŋߖT�_�T����͍ŏ�����
public:
	KDtree(){};
	KDtree& operator=(const KDtree& obj)
	{
		for (const auto& node_list : obj.nodeList)
		{
			Node* node = new Node;
			*node = *node_list;
			this->nodeList.push_back(node);
		}
		this->root = new Node;
		*this->root = *obj.root;

		this->object = obj.object;
		this->referenceData = obj.referenceData;
		this->nearestNeighbor = obj.nearestNeighbor;
		this->distance = obj.distance;
		this->circleMax = obj.circleMax;
		this->circleMin = obj.circleMin;

		this->nearestNeighbor2 = obj.nearestNeighbor2;
		this->distance2 = obj.distance2;
		return *this;
	}

	~KDtree()
	{
		for(std::vector<Node*>::iterator itr(nodeList.begin()); itr != nodeList.end(); ++itr){
			delete *itr;
			*itr = NULL;
		}
	}
private:

	static bool lessCoorx(const Coor<> &rLeft, const Coor<> &rRight) { return rLeft.x < rRight.x; }
	static bool lessCoory(const Coor<> &rLeft, const Coor<> &rRight) { return rLeft.y < rRight.y; }

	void setNode(Node* node, const std::vector<Coor<> >::iterator first, const std::vector<Coor<> >::iterator last, int height)
	{
		Node *p;
		std::vector< Coor<> >::iterator it(first);
		int num(std::distance(first,last));
		int center(static_cast<int>(num/2));
		if((height%2) == 0){
		//X���ŕ���
			std::sort(first,last,lessCoorx);
			std::advance(it,center);
			node->data = it->x;
			if(num > 1){
				p = new Node();
				node->left = p;
				nodeList.push_back(p);
				setNode(p, first, it, height+1);
				p = new Node();
				node->right = p;
				nodeList.push_back(p);
				setNode(p, it, last, height+1);
			}else{
				*node = *it;
			}
		}else{
		//Y���ŕ��� �����͏�Ɠ���
			std::sort(first,last,lessCoory);
			std::advance(it,center);
			node->data = it->y;
			if(num > 1){
				p = new Node();
				node->left = p;
				nodeList.push_back(p);
				setNode(p, first, it, height+1);
				p = new Node();
				node->right = p;
				nodeList.push_back(p);
				setNode(p, it, last, height+1);
			}else{
				*node = *it;
			}
		}
	}
	void refNode(const Node *pNode, int height)
	{
		double dist;
		if((pNode->left != NULL)&&(pNode->right != NULL)){
			if((height%2) == 0){
			//X��
				if(referenceData.x < pNode->data){
					refNode(pNode->left, height+1);
					if(pNode->data <= circleMax.x){
						refNode(pNode->right, height+1);
					}
				}else{
					refNode(pNode->right, height+1);
					if(pNode->data > circleMin.x){
						refNode(pNode->left, height+1);
					}
				}
			}else{
			//Y��
				if(referenceData.y < pNode->data){
					refNode(pNode->left, height+1);
					if(pNode->data <= circleMax.y){
						refNode(pNode->right, height+1);
					}
				}else{
					refNode(pNode->right, height+1);
					if(pNode->data > circleMin.y){
						refNode(pNode->left, height+1);
					}
				}
			}
		}else{
			//Leaf node
			dist = referenceData | pNode->point;//�����̎Z�o
			if(dist < distance){
				circleMin.x = referenceData.x - dist;
				circleMax.x = referenceData.x + dist;
				circleMin.y = referenceData.y - dist;
				circleMax.y = referenceData.y + dist;
				distance = dist;
				nearestNeighbor = pNode->point;
			}
		}

	}
	void refNodef(const Node *pNode, int height)
	{
		if((pNode->left != NULL)&&(pNode->right != NULL)){
			if((height%2) == 0){
			//X��
				if(referenceData.x < pNode->data){
					refNodef(pNode->left , height+1);
				}else{
					refNodef(pNode->right, height+1);
				}
			}else{
			//Y��
				if(referenceData.y < pNode->data){
					refNodef(pNode->left , height+1);
				}else{
					refNodef(pNode->right, height+1);
				}
			}
		}else{
			//Leaf node
			nearestNeighbor = pNode->point;
		}
	}

	void refNode2(const Node *pNode, int height)
	{
		double dist;
		if((pNode->left != NULL)&&(pNode->right != NULL)){
			if((height%2) == 0){
			//X��
				if(referenceData.x < pNode->data){
					refNode2(pNode->left, height+1);
					if(pNode->data <= circleMax.x){
						refNode2(pNode->right, height+1);
					}
				}else{
					refNode2(pNode->right, height+1);
					if(pNode->data > circleMin.x){
						refNode2(pNode->left, height+1);
					}
				}
			}else{
			//Y��
				if(referenceData.y < pNode->data){
					refNode2(pNode->left, height+1);
					if(pNode->data <= circleMax.y){
						refNode2(pNode->right, height+1);
					}
				}else{
					refNode2(pNode->right, height+1);
					if(pNode->data > circleMin.y){
						refNode2(pNode->left, height+1);
					}
				}
			}
		}else{
			//Leaf node
			dist = referenceData | pNode->point;//�����̎Z�o
			if(dist < distance2.first){
				circleMin.x = referenceData.x - distance2.first;
				circleMax.x = referenceData.x + distance2.first;
				circleMin.y = referenceData.y - distance2.first;
				circleMax.y = referenceData.y + distance2.first;
				distance2.second = distance2.first;
				nearestNeighbor2.second = nearestNeighbor2.first;
				distance2.first = dist;
				nearestNeighbor2.first = pNode->point;
			}else if(dist < distance2.second){
				circleMin.x = referenceData.x - dist;
				circleMax.x = referenceData.x + dist;
				circleMin.y = referenceData.y - dist;
				circleMax.y = referenceData.y + dist;
				distance2.second = dist;
				nearestNeighbor2.second = pNode->point;
			}
		}
	}
public:
	void makeTree(const std::vector<Coor<> > &obj)
	{
		for(std::vector<Node*>::iterator i(nodeList.begin()); i != nodeList.end(); ++i){ delete *i; *i = NULL; }
		nodeList.clear();
		object.clear();
		object.insert(object.end(), obj.begin(), obj.end());
		Node *p = new Node();
		root = p;
		nodeList.push_back(p);
		setNode(p, object.begin(), object.end(), 0);
	}
	void makeTree(const std::vector < std::vector<Coor<> > > &obj)
	{
		for(std::vector<Node*>::iterator i(nodeList.begin()); i != nodeList.end(); ++i){ delete *i; *i = NULL; }
		nodeList.clear();
		object.clear();
		for(std::vector <std::vector< Coor<> > >::const_iterator itr(obj.begin()); itr != obj.end(); ++itr){
			object.insert(object.end(), itr->begin(), itr->end());
		}
		Node *p = new Node();
		root = p;
		nodeList.push_back(p);
		setNode(p, object.begin(), object.end(), 0);
	}

	Coor<> searchNeighborPoint(double x, double y)
	{
		referenceData.set(x,y);
		nearestNeighbor.set(HUGE_VAL,HUGE_VAL);
		distance = HUGE_VAL;
		circleMin.set(-HUGE_VAL,-HUGE_VAL);
		circleMax.set(HUGE_VAL,HUGE_VAL);
		refNode(root, 0);
		return nearestNeighbor;
	}
	Coor<> searchNeighborPoint(const Coor<> &obj)
	{
		return searchNeighborPoint(obj.x,obj.y);
	}

	bool findPoint(const Coor<> &obj)
	{
		refNodef(root, 0);
		return (nearestNeighbor == obj);
	}

	double getDistance()const{return distance;}

	std::pair<Coor<>,Coor<> > searchNeighborPoint2(double x, double y)
	{
		referenceData.set(x,y);
		nearestNeighbor2.first.set(HUGE_VAL,HUGE_VAL);
		nearestNeighbor2.second.set(HUGE_VAL,HUGE_VAL);
		distance2.first = HUGE_VAL;
		distance2.second = HUGE_VAL;
		circleMin.set(-HUGE_VAL,-HUGE_VAL);
		circleMax.set(HUGE_VAL,HUGE_VAL);
		refNode2(root, 0);
		return nearestNeighbor2;
	}
	std::pair<Coor<>,Coor<> > searchNeighborPoint2(const Coor<> &obj)
	{
		return searchNeighborPoint2(obj.x,obj.y);
	}
	std::pair<double,double> getDistance2()const{return distance2;}

//���܂� �S�T���ł��c���Ă��� k-D tree �ɖ�肪����΂�����
public:
	Coor<> fullSearch(const Coor<> &obj)
	{
		double dist;
		nearestNeighbor.set(HUGE_VAL,HUGE_VAL);
		distance = HUGE_VAL;
		for(std::vector< Coor<> >::iterator itr(object.begin()); itr != object.end(); ++itr){
			dist = *itr | obj;
			if(dist < distance){
				distance = dist;
				nearestNeighbor = *itr;
			}
		}
		return nearestNeighbor;
	}
};
