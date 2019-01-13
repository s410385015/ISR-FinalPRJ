#include <vector>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace Eigen;
using namespace std;


class Laplacian{

public:
	SparseMatrix<double> LapCoff;
	vector<Vector3d> delta;
	vector<double> orginalScale;

	SparseMatrix<double> eV;
	SparseMatrix<double> NorDelta;

	SimplicialCholesky<SparseMatrix<double>> *chol;
	SimplicialCholesky<SparseMatrix<double>> *NormalDeltaChol;
	

	double getLength(double ax, double ay, double az);
	vector<Triplet<double>> CalcTransform(vector<int> roi, vector<vector<int>> adj, vector<Vector3d> raw, vector<int> roiIIndex);

	vector<Vector3d> CalcDelta(int handleEnd,vector<int> roi, vector<vector<int>> adj, vector<Vector3d> raw, vector<int> roiIndex);

	void PrepareDeform(int handleEnd,vector<int> roi, vector<vector<int>> adj, vector<Vector3d> raw, vector<int> roiIndex);
	MatrixXd DoDeformation(int handleEnd, vector<int> roi, vector<vector<int>> adj, vector<Vector3d> raw,vector<int> roiIIndex);

	
};