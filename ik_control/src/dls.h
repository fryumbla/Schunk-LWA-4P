#include <Eigen/Dense>

using namespace Eigen;

class Joints
{
public:
	Joints(unsigned int numofjoint)
	{
		DoF = numofjoint;
		_q.resize(DoF);
		//_X.resize(6);
		jP.resize(DoF, 3);
		jZ.resize(DoF, 3);
		jY.resize(DoF, 3);
		jX.resize(DoF, 3);
		DH.resize(DoF, 4);
		PC.resize(DoF, 3);
		Mass.resize(DoF);
		Inertia.resize(DoF, 6);
		Hoffset = makeT(0, 0, 0, 0);
		JLimit.resize(2 * DoF);

	};
	void Update(VectorXd* q)
	{
		for (unsigned int i = 0; i<DoF; i++)
			_q(i) = (*q)(i);

		fwk();
	};
	Vector3d GetJPos(int i)
	{
		return jP.row(i);
	};

	Vector3d GetJAxis(int i)
	{
		return jZ.row(i);
	};

	Vector3d GetXPos()
	{
		return _Xp;
	};

	Matrix3d GetXOrient()
	{
		return _Xo;
	};
	Matrix3d GetJR(int i)
	{
		GetR(i);
		return jR;
	};
	Vector3d GetJPc(int i)
	{
		GetPc(i);
		return jPc;
	};
	Matrix4d GetJHT(int i)
	{
		return jHT[i];
	};

	double GetMass(int i)
	{
		UpdateMass();
		return Mass(i);
	};
	unsigned int GetDoF()
	{
		return DoF;
	};
	Matrix4d makeT(double a, double b, double c, double d)
	{
		Matrix4d T;
		T(0, 0) = cos(c);		T(0, 1) = -sin(c);		T(0, 2) = 0;			T(0, 3) = b;
		T(1, 0) = sin(c)*cos(a); T(1, 1) = cos(c)*cos(a); T(1, 2) = -sin(a);	T(1, 3) = -sin(a)*d;
		T(2, 0) = sin(c)*sin(a); T(2, 1) = cos(c)*sin(a); T(2, 2) = cos(a);	T(2, 3) = cos(a)*d;
		T(3, 0) = 0;				T(3, 1) = 0;				T(3, 2) = 0;			T(3, 3) = 1;

		return T;
	};

	VectorXd JLimit;

private:
	unsigned int DoF;
	VectorXd _q;		//current Joint angle
	Vector3d _Xp;		//current position of end effector
	Matrix3d _Xo;		//current orientation of end effector
	MatrixXd jP;		//joint position
	MatrixXd jZ;		//joint axis
	MatrixXd jY;		//joint oppV
	MatrixXd jX;		//joint appV
	Matrix3d jR;		//joint Rotation Matrix
	Vector3d jPc;		//Center of mass of each joints
	MatrixXd DH;		//DH parameter
	Matrix4d Hoffset;
	MatrixXd PC;		//Postion of center of mass of each joints
	VectorXd Mass;
	MatrixXd Inertia;
	Matrix4d jHT[6];		//HT matrix of each joints

	
	void fwk()			//forward kinematics
	{
		UpdateDH();
		Matrix4d H = Matrix4d::Identity();		//transformation matrix
		//H = H * Hoffset;
		for (unsigned int i = 0; i<DoF; i++)
		{
			H = H * makeT(DH(i, 0), DH(i, 1), DH(i, 2), DH(i, 3));
			jHT[i] = H;
			for (int j = 0; j<3; j++)
			{
				jP(i, j) = H(j, 3);
				jZ(i, j) = H(j, 2);
				jY(i, j) = H(j, 1);
				jX(i, j) = H(j, 0);
			}
		}

		Vector4d v, temp;
		v << 0, 0, 0, 1;
		temp = H*v;
		_Xp = temp.head(3);
		_Xo = H.block<3, 3>(0, 0);
	};

	void GetR(int i)
	{
		for (int j = 0; j < 3; j++)
		{
			jR(j, 0) = jX(i, j);
			jR(j, 1) = jY(i, j);
			jR(j, 2) = jZ(i, j);
		}
	};

	


	void GetPc(int i)
	{
		UpdatePC();
		for (int j = 0; j < 3; j++)
		{
			jPc(j) = DH(i, 0)*jX(i, j) + DH(i, 1)*jY(i, j) + DH(i, 2)*jZ(i, j);
		}
	};
	void UpdateDH()
	{
		
		
		DH(0, 0) = M_PI / 2;	DH(0, 1) = 0;		DH(0, 2) = _q(0) + M_PI / 2;	DH(0, 3) = 261.7;	//žÅŽÞ·ÁÀÖÀ»¶§
//		DH(1, 0) = M_PI / 2;	DH(1, 1) = 0;		DH(1, 2) = _q(1) - M_PI / 2;	DH(1, 3) = 0;
		DH(1, 0) = M_PI / 2;	DH(1, 1) = 0;		DH(1, 2) = _q(1);				DH(1, 3) = 0;
		DH(2, 0) = -M_PI / 2;	DH(2, 1) = 0;		DH(2, 2) = _q(2);				DH(2, 3) = 375.7;
		DH(3, 0) = M_PI / 2;	DH(3, 1) = 0;		DH(3, 2) = _q(3);				DH(3, 3) = 0;
		DH(4, 0) = -M_PI / 2;	DH(4, 1) = 0;		DH(4, 2) = _q(4);				DH(4, 3) = 386.2;
		DH(5, 0) = M_PI / 2;	DH(5, 1) = 0;		DH(5, 2) = _q(5);				DH(5, 3) = 0;
		//DH(6, 0) = -M_PI / 2;	DH(6, 1) = -0.1;	DH(6, 2) = 0;					DH(6, 3) = 0.1707;
		


		/*
		DH(0, 0) = 0;	DH(0, 1) = 0;		DH(0, 2) = _q(0) + M_PI / 2;	DH(0, 3) = 261.7;	//Å×ÀÌºí¿¡ ŽÞŸÒÀ»¶§
		DH(1, 0) = M_PI / 2;	DH(1, 1) = 0;		DH(1, 2) = _q(1);				DH(1, 3) = 0;
		DH(2, 0) = -M_PI / 2;	DH(2, 1) = 0;		DH(2, 2) = _q(2);				DH(2, 3) = 375.7;
		DH(3, 0) = M_PI / 2;	DH(3, 1) = 0;		DH(3, 2) = _q(3);				DH(3, 3) = 0;
		DH(4, 0) = -M_PI / 2;	DH(4, 1) = 0;		DH(4, 2) = _q(4);				DH(4, 3) = 386.2;
		DH(5, 0) = M_PI / 2;	DH(5, 1) = 0;		DH(5, 2) = _q(5);				DH(5, 3) = 0;
		*/


	};

	void UpdatePC()//length of vector
	{
		PC(0, 0) = 0.08; PC(0, 1) = 0; PC(0, 2) = 0;
		PC(1, 0) = 0; PC(1, 1) = 0; PC(1, 2) = 0;
		PC(2, 0) = 0.236 / 2; PC(2, 1) = 0; PC(2, 2) = 0;
		PC(3, 0) = 0; PC(3, 1) = 0; PC(3, 2) = 0;
		PC(4, 0) = 0.296 / 2; PC(4, 1) = 0; PC(4, 2) = 0;
		PC(5, 0) = 0; PC(5, 1) = 0; PC(5, 2) = 0;
		PC(6, 0) = 0; PC(6, 1) = 0; PC(6, 2) = 0;
	};

	void UpdateMass()
	{
		Mass << 2.5, 1.232, 1.462, 2.718, 0, 0, 0;
	};
	void UpdateInertia()
	{
		//Ixx,Iyy,Izz,Ixy,Iyz,Izx

	};
};

class Jacobian
{
public:
	Jacobian(unsigned int numofjoint)
	{
		DoF = numofjoint;
		Jg.resize(6, DoF);
		Jinv.resize(DoF, 6);
		damp.resize(7);
		damp << 0.1, 0.1, 0.1, 0.05, 0.01, 0.01, 0.01;
	};
	void Update(Joints* p_joint)
	{
		Jinv.fill(0);
		for (unsigned int i = 0; i<DoF; i++)
		{
			Vector3d v;
			v = p_joint->GetJAxis(i).cross(p_joint->GetXPos() - p_joint->GetJPos(i));
			Jg.block<3, 1>(0, i) = v;
			Jg.block<3, 1>(3, i) = p_joint->GetJAxis(i);
		}

		JacobiSVD<MatrixXd> svd(Jg, ComputeThinU | ComputeThinV);
		MatrixXd U = svd.matrixU();
		MatrixXd V = svd.matrixV();
		VectorXd S = svd.singularValues();

		for (int j = 0; j<S.size(); j++)
		{
			Jinv = Jinv + S(j) / (S(j)*S(j) + damp(j))*V.col(j)*U.transpose().row(j);
		}
	};

	MatrixXd GetJinv()
	{
		return Jinv;
	};

	VectorXd GetTi(Joints* pJoint, int i)
	{
		Ti.resize(i);
		for (int j = 0; j < i; j++)
		{
			Ti(j) = -Jg(2, j)*9.8*pJoint->GetMass(j);
		}
		return Ti;
	};

	unsigned int DoF;
	VectorXd damp;

private:
	MatrixXd Jg;		//geometrical jacobian
	MatrixXd Jinv;		//Jacobian inverse
	Matrix4d _H;		//transformation matrix
	VectorXd Ti;		//i¹øÂ° jointÀÇ ÇÊ¿äÅäÅ©
};



typedef struct
{
public:
	/*double get_appV(int idx) const {

	}
	double& get_appV(int idx){
		return appV[idx];
	}
private:*/
	double appV[3];
	double oppV[3];
	double GoalPos[3];
}Param;