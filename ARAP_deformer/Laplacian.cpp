#include "Laplacian.h"
#include<iostream>
using namespace std;


const int x = 0;
const int y = 1;
const int z = 2;

double Laplacian::getLength(double ax, double ay, double az) {
	return sqrt(ax*ax + ay * ay + az * az);
}
vector<Triplet<double>> Laplacian::CalcTransform(vector<int> roi, vector<vector<int>> adj, vector<Vector3d> raw, vector<int> roiIndex)
{
	
	vector<MatrixXd> Ts;
	Ts.resize(roi.size());
	vector<Triplet<double>> coff;

	for (int i = 0; i < roi.size(); i++)
	{

		
		vector<int> iAndNeighbours;
		iAndNeighbours.push_back(roi[i]);

		for (int j = 0; j < adj[roi[i]].size(); ++j) {
			iAndNeighbours.push_back(adj[roi[i]][j]);
		}


		MatrixXd At(7, iAndNeighbours.size() * 3);
		for (int row = 0; row < 7; ++row) {
			for (int col = 0; col < iAndNeighbours.size() * 3; ++col) {
				At(row, col) = 0.0f;
			}
		}
		for (int j = 0; j < iAndNeighbours.size(); ++j) {
			

			Vector3d vk = raw[iAndNeighbours[j]];
			

			At(0, j * 3 + 0) = +vk[x];
			At(1, j * 3 + 0) = 0;
			At(2, j * 3 + 0) = +vk[z];
			At(3, j * 3 + 0) = -vk[y];
			At(4, j * 3 + 0) = +1;
			At(5, j * 3 + 0) = 0;
			At(6, j * 3 + 0) = 0;

			At(0, j * 3 + 1) = +vk[y];
			At(1, j * 3 + 1) = -vk[z];
			At(2, j * 3 + 1) = 0;
			At(3, j * 3 + 1) = +vk[x];
			At(4, j * 3 + 1) = 0;
			At(5, j * 3 + 1) = +1;
			At(6, j * 3 + 1) = 0;

			At(0, j * 3 + 2) = +vk[z];
			At(1, j * 3 + 2) = +vk[y];
			At(2, j * 3 + 2) = -vk[x];
			At(3, j * 3 + 2) = 0;
			At(4, j * 3 + 2) = 0;
			At(5, j * 3 + 2) = 0;
			At(6, j * 3 + 2) = +1;
		}

		
		MatrixXd AtA = (At*At.transpose()).inverse();
		Ts[i] = AtA*At;

		/*
		//當點是孤立的,inverse可能會是奇異矩陣
		if (iAndNeighbours.size() == 1)
		{
			MatrixXd pseu = MatrixXd::Zero(7, 3);
			Ts[i] = pseu;
		}*/
		

		MatrixXd d(3, 4);
		d(0, 0) = delta[i][x];
		d(0, 1) = 0;
		d(0, 2) = delta[i][z];
		d(0, 3) = -delta[i][y];
		d(1, 0) = delta[i][y];
		d(1, 1) = -delta[i][z];
		d(1, 2) = 0;
		d(1, 3) = delta[i][x];
		d(2, 0) = delta[i][z];
		d(2, 1) = delta[i][y];
		d(2, 2) = -delta[i][x];
		d(2, 3) = 0;

		
		MatrixXd t(4, Ts[i].cols());

		for (int r = 0; r <4 ; r++)
			t.row(r) = Ts[i].row(r);


		MatrixXd tmp = d*t;


		for (int j = 0; j < tmp.cols(); j++){
			for (int k = 0; k < 3; k++){
				int idx = j / 3; 
				int offset = j % 3;
				int verIdx = roiIndex[iAndNeighbours[idx]];
				
				if (k == offset){

					double uniformCoff = LapCoff.coeffRef(i * 3 + k, verIdx * 3 + offset);
					
					coff.push_back(Triplet<double>(i * 3 + k, verIdx * 3 + offset, uniformCoff - tmp(k, j)));
					
				
				}
				else{
					coff.push_back(Triplet<double>(i * 3 + k, verIdx * 3 + offset, -tmp(k, j)));
					
				}
			}
			
		}
		
	}
	
	
	return coff;
	

	
}

vector<Vector3d> Laplacian::CalcDelta(int handleEnd,vector<int> roi, vector<vector<int>> adj, vector<Vector3d> raw,vector<int> roiIndex)
{
	LapCoff.resize(roi.size() * 3, roi.size() * 3);
	vector<Vector3d> delta;
	vector<Triplet<double>> result;
	for (int i = 0; i < roi.size(); i++)
	{
		Vector3d vi = raw[roi[i]];

	
		result.push_back(Triplet<double>((i * 3), (i*3), 1));
		result.push_back(Triplet<double>((i * 3)+1, (i*3)+1, 1));
		result.push_back(Triplet<double>((i * 3)+2, (i*3)+2, 1));
		
		double w = (-1 / (double)adj[roi[i]].size());
		
		for (int j = 0; j < adj[roi[i]].size(); ++j) {	
			vi = vi + w*(raw[adj[roi[i]][j]]);

			int idx = roiIndex[adj[roi[i]][j]];
			result.push_back(Triplet<double>((i * 3), (idx * 3), w));
			result.push_back(Triplet<double>((i * 3) + 1, (idx * 3) + 1, w));
			result.push_back(Triplet<double>((i * 3) + 2, (idx * 3) + 2, w));
		}
		
		
		delta.push_back(vi);
	}
	LapCoff.setFromTriplets(result.begin(), result.end());

	for (int i = 0; i < handleEnd; i++)
	{
		result.push_back(Triplet<double>((roi.size() + i) * 3, i * 3, 1.0));
		result.push_back(Triplet<double>((roi.size() + i) * 3 + 1, i * 3 + 1, 1.0));
		result.push_back(Triplet<double>((roi.size() + i) * 3 + 2, i * 3 + 2, 1.0));
	}
	SparseMatrix<double> augMat((roi.size() + handleEnd) * 3, roi.size() * 3);
	augMat.setFromTriplets(result.begin(), result.end());
	NorDelta = augMat.transpose();
	NormalDeltaChol = new SimplicialCholesky<SparseMatrix<double>>(NorDelta*augMat);
	return delta;
}

void Laplacian::PrepareDeform(int handleEnd,vector<int> roi, vector<vector<int>> adj, vector<Vector3d> raw, vector<int> roiIndex)
{
	
	delta = CalcDelta(handleEnd,roi, adj, raw, roiIndex);
	
	for (int i = 0; i < delta.size(); i++)
		orginalScale.push_back(getLength(delta[i][x], delta[i][y], delta[i][z]));
	
	vector<Triplet<double>> coff = CalcTransform(roi, adj, raw, roiIndex);

	for (int i = 0; i < handleEnd; i++)
	{
		coff.push_back(Triplet<double>((roi.size() +i) * 3, i * 3, 1.0));
		coff.push_back(Triplet<double>((roi.size() +i) * 3 + 1, i * 3 + 1, 1.0));
		coff.push_back(Triplet<double>((roi.size() + i) * 3 + 2, i * 3 + 2, 1.0));
	}

	
	eV.resize((roi.size() + handleEnd) * 3, roi.size() * 3);
	eV.setFromTriplets(coff.begin(), coff.end());
	
	chol = new SimplicialCholesky<SparseMatrix<double>>(eV.transpose()*eV);
	

}
MatrixXd Laplacian::DoDeformation(int handleEnd, vector<int> roi, vector<vector<int>> adj, vector<Vector3d> raw,vector<int> roiIndex){


	MatrixXd b((roi.size() + handleEnd) * 3, 1);
	for (int i = 0; i < roi.size() * 3; i++){
		b(i, 0) = 0.0f;
	}
	for (int i = 0; i < handleEnd; i++)
	{
		Vector3d vi = raw[roi[i]];
		b((roi.size() + i) * 3, 0) = vi[x];
		b((roi.size() + i) * 3+1, 0)=vi[y];
		b((roi.size() + i) * 3+2, 0)=vi[z];
	
	}

	
	MatrixXd v_prime = chol->solve(eV.transpose()*b);

	//return v_prime;
	
	MatrixXd solDelta = LapCoff*v_prime;
	
	
	

	for (int i = 0; i < orginalScale.size(); i++)
	{
		double scale = orginalScale[i] / getLength(solDelta(i * 3, 0), solDelta(i * 3 + 1, 0), solDelta(i * 3 + 2, 0));
		
		for (int j = 0; j < 3; j++){
			b(i*3+j,0) = scale*solDelta(i*3+j,0);
		}
	}
	
	
	MatrixXd normalizedSolution = NormalDeltaChol->solve(NorDelta*b);

	return normalizedSolution;
	
}

