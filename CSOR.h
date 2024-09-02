#ifndef SOR_H
#define SOR_H

#include "CPointCloud.h"
#include "interpolation.h"

class CSOR
{
public:
	/**
	 * Constructor for surface of revolution
	 *
	 * @param pointcloud The given point cloud of the SOR 
	 */
	CSOR(const CPointCloud& pointcloud);
	/**
	 * Extract the generatrix and axis of symmetry of the SOR
	 *
	 * @param curvePoints Output of the curve poly line of the generatrix 
	 * @param symmetricCurvePoints Output of the symmetric polyline to generatrix (might be useful might not, we will see)
	 * @param axis Output of the axis of revolution of the generatrix
	 */
	void ExtractAxisAndGeneratrix(CVector<CPointEx3D>& curvePoints, CVector<CPointEx3D>& symmetricCurvePoints, CPointEx3D& axis);
	/**
	 * Get the transformation matrix from the SOR reconstruction    
	 *
	 * @return The 4x4 transformation matrix.
	 */
	CMatrix<double> GetTransormationMatrix() const;
	/**
	 * Euclidean to cylindrical coordinate transformation
	 *
	 * @param inVector The input vector in euclidean coordinates
	 * @return The output vector to cylindrical coordinates  
	 */
	static CVector<double> EuclideanToCylindrical(const CVector<double>& inVector);
	/**
	 * Cylindrical to Euclidean coordinate transformation
	 *
	 * @param inVector The input vector in cylindrical coordinates
	 * @return The output vector to Euclidean coordinates
	 */
	static CVector<double> CylindricalToEucledian(const CVector<double>& inVector, bool homogenous = true);
	/**
	 * Compute the inverse transformation of a R|T transformation matrix R^t|-R^t(T)
	 *
	 * @param transformationIn The input R|t 4x4 transformation
	 * @return The output inverse transformation R^t|-R^t(T)
	 */
	static CMatrix<double> inverseTransformation(const CMatrix<double>& transformationIn);
	/**
	 * Reconstruct the SOR pointcloud by revolving the generatrix around an axis  
	 *
	 * @param generatrix The generatrix curve of the SOR
	 * @param axis The axis of revolution of the SOR
	 * @param bc The barycenter of the point cloud from which the generatrix was produced
	 * @return The output point cloud of the SOR  
	 */
	static spPointCloud reconstructSOR(const CVector<CPointEx3D>& generatrix, const CPointEx3D& axis, const CPointEx3D& bc);
private:
	// data structure used for sorting 
	struct pnt
	{
		double x;
		double y;
	};
	/**
	 * Sort the points of the (z, rho) points in ascending z
	 * necessary for poly approximation of them
	 *
	 * @param p1 first point
	 * @param p2 second point
	 * @return true if p1.y (z1) < p2.y (z2)
	 */
	static bool sortFunction(pnt p1, pnt p2);
	/**
	 * Find the polygon approximation of the 
	 * necessary for poly approximation of them
	 *
	 * @param p The output polynomial approximation
	 */
	void findAlignedGeneratrix(alglib::barycentricinterpolant& p);
	/**
	 * Find the axis aligned extent in the z-axis
	 * 
	 * @param min The output minimal z value
	 * @param max The output maximal z value 
	 */
	void findAxisAlignedSORExtents(double& min, double& max);
	/**
	 * Find the generatrix poly points, its symmetric poly points and axis of revolution
	 *
	 * @param axis The output axis of revolution
	 * @param curvePoints The output generatrix poly curve points
	 * @param symmetricCurvePoints The output symmetric generatrix poly curve points
	 * @param p The computed polygon approximation
	 */
	void getCurvePointsAndAxis(CPointEx3D& axis, CVector<CPointEx3D>& curvePoints, CVector<CPointEx3D>& symmetricCurvePoints, const alglib::barycentricinterpolant& p);
	/**
	 * Find the best rigid transformation which aligns the ideal polynomial to the datapoints
	 * it stores the rigid transformation to pcaMatrix, i.e.
	 *
	 * @param p The computed polygon approximation
	 * @return The mean squared error of the new approximation
	 */
	double FindBestRigidTransformation(const alglib::barycentricinterpolant& p);
	/**
	 * Find the rotation matrix that rotates a given point around an axis that passes from the 
	 * origin of the coordinate system
	 *
	 * @param angle The angle of rotation in degrees
	 * @param axis The axis of rotation
	 */
	static CMatrix<double> findRotationMatrixAroundAnAxisOfRevolution(double angle, const CPointEx3D& axis);
	/**
	 * Transform the coordinates of cartesianCoordinates to cylindrical coordinates and store
	 * to cylindricalCoordinates
	 */
	void getCylindricalCoordinates();
	/**
	 * Transform the coordinates of cylindricalCoordinates to Euclidean coordinates and store
	 * to cartesianCoordinates
	 */
	void getCartesianCoordinates();
	CMatrix<double> pcaMatrix; // The PCA matrix which aligns to the z-axis the SOR
	CMatrix<double> icpMatrix; // The ICP matrix which corrects the SOR alignment
	CVector<CVector<double>> cylindricalCoordinates; // The current cylindrical coordinates of the SOR
	CVector<CVector<double>> cartesianCoordinates;   // The current Euclidean coordinates of the SOR
};

#endif

