// SFML_matrixtransform.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <vector>

struct point
{
	float X, Y, Z;
};

template <typename T> int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

template <unsigned int x, unsigned int y>
void PrintMatrix(const float(&Matrix)[x][y])
{
	int i, j;
	for (i = 0; i < x; i++)
	{
		for (j = 0; j < y; j++)
			std::cout << Matrix[i][j] << '\t';
		std::cout << '\n';
	}
};

template <unsigned int x>
void PrintVect(const float(&Vect)[x])
{
	int i;
	for (i = 0; i < x; i++)
	{
		std::cout << Vect[i] << '\t';
		std::cout << '\n';
	}
};

float determinantOfMatrix(const float(&mat)[3][3])
{
	double ans;
	ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
		- mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
		+ mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
	return ans;
}

// This function finds the solution of system of
// linear equations using cramer's rule
void findSolution(const float(&coeff)[3][3], float(&vect)[3])
{
	// Matrix d using coeff as given in cramer's rule
	float d[3][3] = {
		{ coeff[0][0], coeff[0][1], coeff[0][2] },
		{ coeff[1][0], coeff[1][1], coeff[1][2] },
		{ coeff[2][0], coeff[2][1], coeff[2][2] },
	};
	// Matrix d1 using coeff as given in cramer's rule
	float d1[3][3] = {
		{ vect[0], coeff[0][1], coeff[0][2] },
		{ vect[1], coeff[1][1], coeff[1][2] },
		{ vect[2], coeff[2][1], coeff[2][2] },
	};
	// Matrix d2 using coeff as given in cramer's rule
	float d2[3][3] = {
		{ coeff[0][0], vect[0], coeff[0][2] },
		{ coeff[1][0], vect[1], coeff[1][2] },
		{ coeff[2][0], vect[2], coeff[2][2] },
	};
	// Matrix d3 using coeff as given in cramer's rule
	float d3[3][3] = {
		{ coeff[0][0], coeff[0][1], vect[0]},
		{ coeff[1][0], coeff[1][1], vect[1]},
		{ coeff[2][0], coeff[2][1], vect[2]},
	};

	// Calculating Determinant of Matrices d, d1, d2, d3
	float D = determinantOfMatrix(d);
	float D1 = determinantOfMatrix(d1);
	float D2 = determinantOfMatrix(d2);
	float D3 = determinantOfMatrix(d3);

	// Case 1
	if (D != 0)
	{
		// Coeff have a unique solution. Apply Cramer's Rule
		double x = D1 / D;
		double y = D2 / D;
		double z = D3 / D; // calculating z using cramer's rule

		vect[0] = x;
		vect[1] = y;
		vect[2] = z;
	}
	// Case 2
	else {
		if (D1 == 0 && D2 == 0 && D3 == 0)
			std::cout << ("Infinite solutions\n");
		else if (D1 != 0 || D2 != 0 || D3 != 0)
			std::cout << ("No solutions\n");
	}
}


template <unsigned int x, unsigned int y>
void GetTransformMMatrix(const std::vector<point>& Set1, const std::vector<point> Set2, float(&Matrix)[x][y])
{
	std::cout << "log" << '\n';
	if (x != 4 || y != 4)
	{
		std::cerr << "the matrix shold be 4*4!";
		assert(false);
	}

	// construct X matrixix
	Eigen::Matrix4f OriginMatrix;

	OriginMatrix << Set1[0].X, Set1[1].X, Set1[2].X, Set1[3].X,
		Set1[0].Y, Set1[1].Y, Set1[2].Y, Set1[3].Y,
		Set1[0].Z, Set1[1].Z, Set1[2].Z, Set1[3].Z,
		1, 1, 1, 1;

	std::cout << "x matrix:\n" << OriginMatrix << std::endl;

	Eigen::Vector4f X_vect;
	X_vect << Set2[0].X, Set2[1].X, Set2[2].X, Set2[3].X;
	std::cout << "x vect:\n" << X_vect << std::endl;

	Eigen::Vector4f Res14 = OriginMatrix.colPivHouseholderQr().solve(X_vect);

	// construct Y matrix
	Eigen::Vector4f Y_vect;
	Y_vect << Set2[0].Y, Set2[1].Y, Set2[2].Y, Set2[3].Y;
	std::cout << "y vect:\n" << Y_vect << std::endl;

	Eigen::Vector4f Res24 = OriginMatrix.colPivHouseholderQr().solve(Y_vect);

	// construct Z matrix
	Eigen::Vector4f Z_vect;
	Z_vect << Set2[0].Z, Set2[1].Z, Set2[2].Z, Set2[3].Z;
	std::cout << "z vect:\n" << Z_vect << std::endl;

	Eigen::Vector4f Res34 = OriginMatrix.colPivHouseholderQr().solve(Z_vect);

	// Construct resulted matrix
	int i, j;
	for (i = 0; i < 4; i++)
	{
		Matrix[i][0] = Res14[i];
		Matrix[i][1] = Res24[i];
		Matrix[i][2] = Res34[i];
		Matrix[i][3] = 0;
	}
	Matrix[3][3] = 1;

	return;
}


void VectorMatrixMultiply(const std::vector<point>& Set1, float(&Matrix)[3][3], std::vector<point>& Set2)
{
	Set2.clear();
	for (auto it : Set1)
	{
		point Point;
		Point.X = it.X * Matrix[0][0] + it.Y * Matrix[1][0] + it.Z * Matrix[2][0];
		Point.Y = it.X * Matrix[0][1] + it.Y * Matrix[1][1] + it.Z * Matrix[2][1];
		Point.Z = it.X * Matrix[0][2] + it.Y * Matrix[1][2] + it.Z * Matrix[2][2];
		Set2.push_back(Point);
	}/**/
	PrintMatrix(Matrix);

	
}

template <unsigned int x, unsigned int y>
void GetRotationMatrix(const std::vector<point>& Set1, const std::vector<point>& Set2, float(&Matrix)[x][y])
{
	std::cout << "log" << '\n';
	if (x != 3 || y != 3)
	{
		std::cerr << "the matrix shold be 3*3!";
		assert(false);
	}


	// construct X matrixix
	Eigen::Matrix3f OriginMatrix1;

	OriginMatrix1 << Set1[0].X, Set1[0].Y, Set1[0].Z,
		Set1[1].X, Set1[1].Y, Set1[1].Z,
		Set1[2].X, Set1[2].Y, Set1[2].Z;


	float MatrixNew[3][3] =
	{
		Set1[0].X, Set1[0].Y, Set1[0].Z,
		Set1[1].X, Set1[1].Y, Set1[1].Z,
		Set1[2].X, Set1[2].Y, Set1[2].Z,
	};

	float  Vect[3] = { Set2[0].X, Set2[1].X, Set2[2].X };
	findSolution(MatrixNew, Vect);
	float  Vect2[3] = { Set2[0].Y, Set2[1].Y, Set2[2].Y };
	findSolution(MatrixNew, Vect2);
	float  Vect3[3] = { Set2[0].Z, Set2[1].Z, Set2[2].Z };
	findSolution(MatrixNew, Vect3);



	float MatrixX[3][3] = { Vect[0], Vect2[0], Vect3[0],
							Vect[1], Vect2[1], Vect3[1],
							Vect[2], Vect2[2], Vect3[2] };

	// Construct resulted matrix
	int i;
	for (i = 0; i < 3; i++)
	{
		Matrix[i][0] = Vect[i];
		Matrix[i][1] = Vect2[i];
		Matrix[i][2] = Vect3[i];
	}

	return;
}

int main()
{
	point A = { 100, 100, 1 };
	point A1 = { 50+150, 150, 1 };

	point B = { 200, 100, 1 };
	point B1 = { 150 + 150, 50, 1 };

	point C = { 200, 200, 1 };
	point C1 = { 250 + 150, 150, 1 };

	point D = { 150, 250, 1 };
	point D1 = { -2, -1, 1 };

	point E = { 100, 200, 1 };

	std::vector<point> PointVector1;

	PointVector1.push_back(std::move(A));
	PointVector1.push_back(std::move(B));
	PointVector1.push_back(std::move(C));
	PointVector1.push_back(std::move(D));
	PointVector1.push_back(std::move(E));

	std::vector<point> PointVector2;
	PointVector2.push_back(std::move(A1));
	PointVector2.push_back(std::move(B1));
	PointVector2.push_back(std::move(C1));
	float M[3][3];
	GetRotationMatrix(PointVector1, PointVector2, M);
	VectorMatrixMultiply(PointVector1,M, PointVector2);

    sf::RenderWindow window(sf::VideoMode(600, 600), "SFML works!");


     sf::ConvexShape convex, convex_after_transform;
    convex.setPointCount(5);
	convex_after_transform.setPointCount(5);
    // define the points
	int i;
	for (i = 0; i < 5; i++)
	{
		convex.setPoint(i, sf::Vector2f(PointVector1[i].X, PointVector1[i].Y));
		convex_after_transform.setPoint(i, sf::Vector2f(PointVector2[i].X, PointVector2[i].Y));
		std::cout << "Point  " << PointVector2[i].X <<'\n';
	}
    
	convex_after_transform.setFillColor(sf::Color::Green);
    convex.setFillColor(sf::Color::Red);
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(convex);
		window.draw(convex_after_transform);
        window.display();
    }
   


    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
