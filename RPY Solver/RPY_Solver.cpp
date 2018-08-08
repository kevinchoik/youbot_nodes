#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <math.h>

int main(int argc, char const *argv[])
{
	// Identity matrix
	double matrix [3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	while(true) {
		std::cout << "> ";
		std::string inputStr;
		std::getline(std::cin, inputStr);
		std::istringstream inputSs(inputStr);
		std::vector<std::string> inputVec;
		for (std::string s; inputSs >> s; ) {
			inputVec.push_back(s);
		}
		if (inputVec.size() == 2) {
			double rad = atof(inputVec[1].c_str()) * M_PI / 180.0;
			double temp [3][3];
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					temp[i][j] = matrix[i][j];
				}
			}
			if (inputVec[0] == "x") {
				// Rotation matrix multiplication in the x axis
				temp[0][0] = matrix[0][0];
				temp[0][1] = matrix[0][1];
				temp[0][2] = matrix[0][2];
				temp[1][0] = cos(rad) * matrix[1][0] - sin(rad) * matrix[2][0];
				temp[1][1] = cos(rad) * matrix[1][1] - sin(rad) * matrix[2][1];
				temp[1][2] = cos(rad) * matrix[1][2] - sin(rad) * matrix[2][2];
				temp[2][0] = sin(rad) * matrix[1][0] + cos(rad) * matrix[2][0];
				temp[2][1] = sin(rad) * matrix[1][1] + cos(rad) * matrix[2][1];
				temp[2][2] = sin(rad) * matrix[1][2] + cos(rad) * matrix[2][2];
			} else if (inputVec[0] == "y") {
				// Rotation matrix multiplication in the y axis
				temp[0][0] = cos(rad) * matrix[0][0] + sin(rad) * matrix[2][0];
				temp[0][1] = cos(rad) * matrix[0][1] + sin(rad) * matrix[2][1];
				temp[0][2] = cos(rad) * matrix[0][2] + sin(rad) * matrix[2][2];
				temp[1][0] = matrix[1][0];
				temp[1][1] = matrix[1][1];
				temp[1][2] = matrix[1][2];
				temp[2][0] = -sin(rad) * matrix[0][0] + cos(rad) * matrix[2][0];
				temp[2][1] = -sin(rad) * matrix[0][1] + cos(rad) * matrix[2][1];
				temp[2][2] = -sin(rad) * matrix[0][2] + cos(rad) * matrix[2][2];
			} else if (inputVec[0] == "z") {
				// Rotation matrix multiplication in the z axis
				temp[0][0] = cos(rad) * matrix[0][0] - sin(rad) * matrix[1][0];
				temp[0][1] = cos(rad) * matrix[0][1] - sin(rad) * matrix[1][1];
				temp[0][2] = cos(rad) * matrix[0][2] - sin(rad) * matrix[1][2];
				temp[1][0] = sin(rad) * matrix[0][0] + cos(rad) * matrix[1][0];
				temp[1][1] = sin(rad) * matrix[0][1] + cos(rad) * matrix[1][1];
				temp[1][2] = sin(rad) * matrix[0][2] + cos(rad) * matrix[1][2];
				temp[2][0] = matrix[2][0];
				temp[2][1] = matrix[2][1];
				temp[2][2] = matrix[2][2];
			}
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					matrix[i][j] = temp[i][j];
				}
			}
		} else if (inputVec.size() == 1) {
			// Compute to RPY
			if (inputVec[0] == "Solve" || inputVec[0] == "solve" || inputVec[0] == "s") {
				// Solution is not unique, so vector used to display all solutions
				std::vector<double> alpha, beta, gamma;
				for (int i = 0; i < 8; i++) {
					double atemp = atan(matrix[2][1] / matrix[2][2]) + M_PI * (i % 2);
					double btemp = asin(-matrix[2][0]);
					if ((i / 2) % 2 == 1) {
						btemp = M_PI - btemp;
					}
					double gtemp = atan(matrix[1][0] / matrix[0][0]) + M_PI * (i / 4);
					// Matrix with the temporary angles
					double check [3][3] = {{cos(gtemp) * cos(btemp), cos(gtemp) * sin(btemp) * sin(atemp) - sin(gtemp) * cos(atemp), cos(gtemp) * sin(btemp) * cos(atemp) + sin(gtemp) * sin(atemp)}, {sin(gtemp) * cos(btemp), sin(gtemp) * sin(btemp) * sin(atemp) + cos(gtemp) * cos(atemp), sin(gtemp) * sin(btemp) * cos(atemp) - cos(gtemp) * sin(atemp)}, {-sin(btemp), cos(btemp) * sin(atemp), cos(btemp) * cos(atemp)}};
					// Matrix difference (2-norm)
					double diff = 0;
					for (int j = 0; j < 3; j++) {
						for (int k = 0; k < 3; k++) {
							diff += pow(matrix[j][k] - check[j][k], 2);
						}
					}
					if (diff < 0.0000001) {
						alpha.push_back(atemp);
						beta.push_back(btemp);
						gamma.push_back(gtemp);
					}
				}
				for (int i = 0; i < alpha.size(); i++) {
					std::cout << "X/R: " << alpha[i] * 180.0 / M_PI << "\nY/P: " << beta[i] * 180.0 / M_PI << "\nZ/Y: " << gamma[i] * 180.0 / M_PI << "\n-----\n";
				}
				// Debugging - Prints entire matrix
				//std::cout << matrix[0][0] << ", " << matrix[0][1] << ", " <<matrix[0][2] << "\n" << matrix[1][0] << ", " << matrix[1][1] << ", " <<matrix[1][2] << "\n" << matrix[2][0] << ", " << matrix[2][1] << ", " <<matrix[2][2] << "\n";
			} else if (inputVec[0] == "Reset" || inputVec[0] == "reset" || inputVec[0] == "r") {
				// Reset to identity matrix
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						if (i == j) {
							matrix[i][j] = 1;
						} else {
							matrix[i][j] = 0;
						}
					}
				}
			}
		}
	}
	return 0;
}