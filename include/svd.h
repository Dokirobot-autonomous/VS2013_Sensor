//特異値分解用ヘッダ
//NUMERICAL RECIPES in C より
//動的配列をvectorに変更
#pragma once

#include <cmath>
#include <vector>

class nrC{

	//static float sqrarg;
	//#define SQR(a) ((sqrarg = (a)) == 0.0 ? 0.0 : sqrarg*sqrarg)

	//static double dsqrarg;
//#define DSQR(a) ((dsqrarg = (a)) == 0.0 ? 0.0 : dsqrarg*dsqrarg)
	static double DSQR(double a) { return a == 0.0 ? 0.0 : a*a; };

	//	static double dmaxarg1, dmaxarg2;
	static double DMAX(double a, double b) { return a > b ? a : b; };
	//#define DMAX(a,b) (dmaxarg1 = (a), dmaxarg2 = (b), (dmaxarg1) > (dmaxarg2) ? (dmaxarg1) : (dmaxarg2))

	//static double dminarg1,dminarg2;
	//#define DMIN(a,b) (dminarg1 = (a), dminarg2 = (b), (dminarg1) < (dminarg2) ? (dminarg1) : (dminarg2))

	//static float maxarg1,maxarg2;
	//#define FMAX(a,b) (maxarg1 = (a), maxarg2 = (b), (maxarg1) > (maxarg2) ? (maxarg1) : (maxarg2))

	//static float minarg1,minarg2;
	//#define FMIN(a,b) (minarg1 = (a), minarg2 = (b), (minarg1) < (minarg2) ? (minarg1) : (minarg2))

	//static long lmaxarg1,lmaxarg2;
	//#define LMAX(a,b) (lmaxarg1 = (a), lmaxarg2 = (b), (lmaxarg1) > (lmaxarg2) ? (lmaxarg1) : (lmaxarg2))

	//static long lminarg1,lminarg2;
	//#define LMIN(a,b) (lminarg1 = (a), lminarg2 = (b), (lminarg1) < (lminarg2) ? (lminarg1) : (lminarg2))

	//static int imaxarg1,imaxarg2;
	//#define IMAX(a,b) (imaxarg1 = (a), imaxarg2 = (b), (imaxarg1) > (imaxarg2) ? (imaxarg1) : (imaxarg2))

	//static int iminarg1, iminarg2;
	//#define IMIN(a,b) (iminarg1 = (a), iminarg2 = (b), (iminarg1) < (iminarg2) ? (iminarg1) : (iminarg2))
	static double IMIN(double a, double b) { return a < b ? a : b; };

//#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
	static double SIGN(double a, double b) { return b >= 0.0 ? fabs(a) : -fabs(a); };


	static double pythag(double a, double b)
		//(a^{2}+b^{2})^{1/2}を計算する。悪いアンダーフローやオーバーフローをしない。
	{
		double absa, absb;
		absa = fabs(a);
		absb = fabs(b);
		if (absa > absb) return absa*sqrt(1.0 + DSQR(absb / absa));
		else return absb == 0.0 ? 0.0 : absb*sqrt(1.0 + DSQR(absa / absb));
	}


	//void nrerror(char error_text[])
	///* Numerical Recipes standard error handler */
	//{
	//	  fprintf(stderr, "Numerical Recipes run-time error...\n");
	//	  fprintf(stderr, "%s\n", error_text);
	//	  fprintf(stderr, "...now exiting to system...\n");
	//	  exit(1);
	//}

public:
	static bool svdcmp(std::vector<std::vector<double>> &a, int m, int n, std::vector<double> &w, std::vector<std::vector<double>> &v)
		//aはa[1..m][1..n]
	{
		//double pythag(double a, double b);
		int flag, i, its, j, jj, k, l, nm;
		double anorm, c, f, g, h, s, scale, x, y, z;
		std::vector<double> rv1(n + 1);

		g = scale = anorm = 0.0;	//Householder法で2重対角の形に直す
		for (i = 1; i <= n; i++)
		{
			l = i + 1;
			rv1[i] = scale*g;
			g = s = scale = 0.0;
			if (i <= m)
			{
				for (k = i; k <= m; k++) scale += fabs(a[k][i]);
				if (scale)
				{
					for (k = i; k <= m; k++)
					{
						a[k][i] /= scale;
						s += a[k][i] * a[k][i];
					}
					f = a[i][i];
					g = (-SIGN(sqrt(s), f));
					h = f*g - s;
					a[i][i] = f - g;
					for (j = l; j <= n; j++)
					{
						for (s = 0.0, k = i; k <= m; k++) s += a[k][i] * a[k][j];
						f = s / h;
						for (k = i; k <= m; k++) a[k][j] += f*a[k][i];
					}
					for (k = i; k <= m; k++) a[k][i] *= scale;
				}
			}
			w[i] = scale*g;
			g = s = scale = 0.0;
			if (i <= m && i != n)
			{
				for (k = l; k <= n; k++) scale += fabs(a[i][k]);
				if (scale)
				{
					for (k = l; k <= n; k++)
					{
						a[i][k] /= scale;
						s += a[i][k] * a[i][k];
					}
					f = a[i][l];
					g = (-SIGN(sqrt(s), f));
					h = f*g - s;
					a[i][l] = f - g;
					for (k = l; k <= n; k++) rv1[k] = a[i][k] / h;
					for (j = l; j <= m; j++)
					{
						for (s = 0.0, k = l; k <= n; k++) s += a[j][k] * a[i][k];
						for (k = l; k <= n; k++) a[j][k] += s*rv1[k];
					}
					for (k = l; k <= n; k++) a[i][k] *= scale;
				}
			}
			anorm = DMAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
		}
		for (i = n; i >= 1; i--)	 //右側変換の累積
		{
			if (i < n)
			{
				if (g)
				{
					for (j = l; j <= n; j++)	 //アンダーフローを避けるため，2度割りする
					{
						v[j][i] = (a[i][j] / a[i][l]) / g;
					}
					for (j = l; j <= n; j++)
					{
						for (s = 0.0, k = l; k <= n; k++) s += a[i][k] * v[k][j];
						for (k = l; k <= n; k++) v[k][j] += s*v[k][i];
					}
				}
				for (j = l; j <= n; j++) v[i][j] = v[j][i] = 0.0;
			}
			v[i][i] = 1.0;
			g = rv1[i];
			l = i;
		}
		for (i = IMIN(m, n); i >= 1; i--)	 //左側変換の累積
		{
			l = i + 1;
			g = w[i];
			for (j = l; j <= n; j++) a[i][j] = 0.0;
			if (g)
			{
				g = (float)(1.0 / g);
				for (j = l; j <= n; j++)
				{
					for (s = 0.0, k = l; k <= m; k++) s += a[k][i] * a[k][j];
					f = (s / a[i][i])*g;
					for (k = i; k <= m; k++) a[k][j] += f*a[k][i];
				}
				for (j = i; j <= m; j++) a[j][i] *= g;
			}
			else for (j = i; j <= m; j++) a[j][i] = 0.0;
			++a[i][i];
		}
		for (k = n; k >= 1; k--)	 //2重対角行列の対角比：特異値についてのループと，
		{
			for (its = 1; its <= 30; its++)	//反復計算のループ
			{
				flag = 1;
				for (l = k; l >= 1; l--)	 //分割のチェック
				{
					nm = l - 1;	 //rv1[1]は常に0
					if (fabs(rv1[l]) + anorm == anorm)
					{
						flag = 0;
						break;
					}
					if (fabs(w[nm]) + anorm == anorm) break;
				}
				if (flag)
				{
					c = 0.0;	//l>1のときrv1[l]を消去
					s = 1.0;
					for (i = l; i <= k; i++)
					{
						f = s*rv1[i];
						rv1[i] = c*rv1[i];
						if (fabs(f) + anorm == anorm) break;
						g = w[i];
						h = pythag(f, g);
						w[i] = h;
						h = 1.0 / h;
						c = g*h;
						s = -f*h;
						for (j = 1; j <= m; j++)
						{
							y = a[j][nm];
							z = a[j][i];
							a[j][nm] = y*c + z*s;
							a[j][i] = z*c - y*s;
						}
					}
				}
				z = w[k];
				if (l == k)	//収束した特異値を非負にする
				{
					if (z < 0.0)
					{
						w[k] = -z;
						for (j = 1; j <= n; j++) v[j][k] = -v[j][k];
					}
					break;
				}
				if (its == 30) return false;	 //30回繰り返しても収束しなかった
				x = w[l];	 //最下部の2行2列の小行列からのシフト
				nm = k - 1;
				y = w[nm];
				g = rv1[nm];
				h = rv1[k];
				f = ((y - z)*(y + z) + (g - h)*(g + h)) / (2.0*h*y);
				g = pythag(f, 1.0);
				f = ((x - z)*(x + z) + h*((y / (f + SIGN(g, f))) - h)) / x;
				c = s = 1.0;	//次のＱＲ変換
				for (j = l; j <= nm; j++)
				{
					i = j + 1;
					g = rv1[i];
					y = w[i];
					h = s*g;
					g = c*g;
					z = pythag(f, h);
					rv1[j] = z;
					c = f / z;
					s = h / z;
					f = x*c + g*s;
					g = g*c - x*s;
					h = y*s;
					y *= c;
					for (jj = 1; jj <= n; jj++)
					{
						x = v[jj][j];
						z = v[jj][i];
						v[jj][j] = x*c + z*s;
						v[jj][i] = z*c - x*s;
					}
					z = pythag(f, h);
					w[j] = z;	 //z=0なら回転は任意
					if (z)
					{
						z = 1.0 / z;
						c = f*z;
						s = h*z;
					}
					f = c*g + s*y;
					x = c*y - s*g;
					for (jj = 1; jj <= m; jj++)
					{
						y = a[jj][j];
						z = a[jj][i];
						a[jj][j] = y*c + z*s;
						a[jj][i] = z*c - y*s;
					}
				}
				rv1[l] = 0.0;
				rv1[k] = f;
				w[k] = x;
			}
		}
		return true;
	};

};
