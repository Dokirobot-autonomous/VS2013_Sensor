//---再近傍点探索クラスVer.4.1---//
//説明:
//      k-D tree により実装
//      対象はx-yの直交座標系上の点 (2次元)
//内容:
//      makeTree を実行するとk-D木が作られる (既存の木に新たにデータを追加する機能は無い ※今のところ作り直しで対応)
//      searchNeighborPoint 最近傍点を返す
//      findPoint 木の中に指定したデータがある場合 true を返す
//      getDistance 最近傍点との距離を返す
//前回からの変更:
//      Ver.4.0: 説明の追加, 微調整
//      Ver.4.1: 再近傍点とその次に近い点を返すようにした関数を追加 (2012:11:27)


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
	std::vector<Node*> nodeList;//delete用
	Node* root;//根
	std::vector< Coor<> > object;//木にする対象
	Coor<> referenceData;//最近傍点探索時の基準
	Coor<> nearestNeighbor;//最近傍点
	double distance;//距離 最近傍点探索後は最小距離
	Coor<> circleMin,circleMax;//最近傍点探索時の基準を中心とする円の範囲

	//近傍二点を取る
	std::pair<Coor<>,Coor<> > nearestNeighbor2;//最近傍点
	std::pair<double,double> distance2;//距離 最近傍点探索後は最小距離
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
		//X軸で分割
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
		//Y軸で分割 処理は上と同一
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
			//X軸
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
			//Y軸
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
			dist = referenceData | pNode->point;//距離の算出
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
			//X軸
				if(referenceData.x < pNode->data){
					refNodef(pNode->left , height+1);
				}else{
					refNodef(pNode->right, height+1);
				}
			}else{
			//Y軸
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
			//X軸
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
			//Y軸
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
			dist = referenceData | pNode->point;//距離の算出
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

//おまけ 全探索版も残しておく k-D tree に問題があればこっち
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
