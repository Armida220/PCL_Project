#include "NearestSearch.h"
#include <iostream>
NearestSearch::NearestSearch(void):m_dataset(NULL),m_num_of_cloud(0)
{

}

NearestSearch::~NearestSearch(void)
{
	if (m_dataset != NULL)
	{
		delete m_dataset ;
		m_dataset = NULL ;
	}
}

//输入点云数据
int NearestSearch::setInputCloud(std::vector<new3s_PointXYZ> &cloud)
{
	m_num_of_cloud = cloud.size() ;
	if (m_num_of_cloud == 0)
	{
#ifdef _DEBUG
		std::cout<<"输入点云数据为空！\n" ;
#endif // _DEBUG
		return 0 ;
	}
	m_dataset = new float[3 * m_num_of_cloud] ;
	for (int i = 0 ; i < m_num_of_cloud ; ++i)
	{
		m_dataset[3 * i + 0] = cloud[i].get_x() ;
		m_dataset[3 * i + 1] = cloud[i].get_y() ;
		m_dataset[3 * i + 2] = cloud[i].get_z() ;
	}
	flann::Matrix<float> dataset = flann::Matrix<float>(m_dataset , m_num_of_cloud , 3) ;
	m_index.reset(new FLANNIndex(dataset , ::flann::KDTreeSingleIndexParams(15))) ;
	m_index->buildIndex() ;
	return 1 ;
}

//KNN点云邻域搜索
int NearestSearch::nearestKSearch(new3s_PointXYZ &query_point , const int &knn , std::vector<new3s_PointXYZ> &indice_points , std::vector<float> &distance)
{
	if (m_num_of_cloud == 0)
	{
#ifdef _DEBUG
		std::cout<<"没有输入点云数据，无法执行最近邻搜索！\n" ;
		return 0 ;
#endif // _DEBUG
	}

	float *query = new float[3] ;
	query[0] = query_point.get_x() ;
	query[1] = query_point.get_y() ;
	query[2] = query_point.get_z() ;

	flann::Matrix<float> queryset = flann::Matrix<float>(query , 1 , 3) ;
// 	index.buildIndex() ;

	std::vector<int> indices(knn) ; //保存查询点的索引
	distance.resize(knn) ;
	flann::Matrix<int> indice_mat(&indices[0] , 1 , knn) ;
	flann::Matrix<float> dist_mat(&distance[0] , 1 , knn) ;
	
	int return_value = m_index->knnSearch(queryset , indice_mat , dist_mat , knn , flann::SearchParams(15)) ;
	for (int i = 0 ; i < knn ; ++i)
	{
        indice_points.push_back(new3s_PointXYZ(m_dataset[3 * indices[i] + 0] , m_dataset[3 * indices[i] + 1] , m_dataset[3 * indices[i] + 2])) ;
	}
	return return_value ;
}

int NearestSearch::nearestKSearch(new3s_PointXYZ &query_point , const int &knn , std::vector<int> &indices , std::vector<float> &distance)
{
	if (m_num_of_cloud == 0)
	{
#ifdef _DEBUG
		std::cout<<"没有输入点云数据，无法执行最近邻搜索！\n" ;
		return 0 ;
#endif // _DEBUG
	}

	float *query = new float[3] ;
	query[0] = query_point.get_x() ;
	query[1] = query_point.get_y() ;
	query[2] = query_point.get_z() ;

	flann::Matrix<float> queryset = flann::Matrix<float>(query , 1 , 3) ;
	// 	index.buildIndex() ;

	indices.resize(knn) ; //保存查询点的索引
	distance.resize(knn) ;
	flann::Matrix<int> indice_mat(&indices[0] , 1 , knn) ;
	flann::Matrix<float> dist_mat(&distance[0] , 1 , knn) ;

	int return_value = m_index->knnSearch(queryset , indice_mat , dist_mat , knn , flann::SearchParams(15)) ;
	return return_value ;
}

//RNN点云邻域搜索
int NearestSearch::nearestRSearch(new3s_PointXYZ &query_point , const float &radius , std::vector<new3s_PointXYZ> &indice_points , std::vector<float> &distance)
{
	if (m_num_of_cloud == 0)
	{
#ifdef _DEBUG
		std::cout<<"没有输入点云数据，无法执行最近邻搜索！\n" ;
		return 0 ;
#endif // _DEBUG
	}
	std::vector<vecInt> radius_indices(1) ;
	std::vector<vecFloat> radius_dists(1) ;

	float *query = new float[3] ;
	query[0] = query_point.get_x() ;
	query[1] = query_point.get_y() ;
	query[2] = query_point.get_z() ;

    flann::Matrix<float> queryset = flann::Matrix<float>(query , 1 , 3) ;

	int neighbors_in_radius = m_index->radiusSearch(queryset , radius_indices , radius_dists , radius , flann::SearchParams(15)) ;
	distance = radius_dists[0] ;
	for (int i = 0 ; i < radius_indices[0].size() ; ++i)
	{
		indice_points.push_back(new3s_PointXYZ(m_dataset[3 * radius_indices[0][i] + 0] , m_dataset[3 * radius_indices[0][i] + 1] , m_dataset[3 * radius_indices[0][i] + 2])) ;
	}
    
	return neighbors_in_radius ;
}

int NearestSearch::nearestRSearch(new3s_PointXYZ &query_point , const float &radius , std::vector<int> &indices , std::vector<float> &distance)
{
	if (m_num_of_cloud == 0)
	{
#ifdef _DEBUG
		std::cout<<"没有输入点云数据，无法执行最近邻搜索！\n" ;
		return 0 ;
#endif // _DEBUG
	}
	std::vector<vecInt> radius_indices(1) ;
	std::vector<vecFloat> radius_dists(1) ;

	float *query = new float[3] ;
	query[0] = query_point.get_x() ;
	query[1] = query_point.get_y() ;
	query[2] = query_point.get_z() ;

    flann::Matrix<float> queryset = flann::Matrix<float>(query , 1 , 3) ;

	int neighbors_in_radius = m_index->radiusSearch(queryset , radius_indices , radius_dists , radius , flann::SearchParams(15)) ;
	indices = radius_indices[0] ;
	distance = radius_dists[0] ;

	return neighbors_in_radius ;
}

//设置点云搜索方法和参数
int NearestSearch::setNearestSearchParameters(const float &parameter , SearchMethod method)
{
	if (method == 0)
	{
		m_search_number = parameter ;
		m_search_method = method ;
	}
	else if (method == 1)
	{
		m_search_radius = parameter ;
		m_search_method = method ;
	}	
	return 1 ;
}

int NearestSearch::setNearestSearchParameters(const float &parameter1 , const float &parameter2 , SearchMethod method)
{
	if (method == 2)
	{
		m_search_radius = parameter1 ;
		m_search_rz = parameter2 ;
		m_search_method = method ;
		float *dataset2D = new float[3 * m_num_of_cloud] ;
		for (int i = 0 ; i < m_num_of_cloud ; ++i)
		{
			dataset2D[3 * i + 0] = m_dataset[3 * i + 0] ;
			dataset2D[3 * i + 1] = m_dataset[3 * i + 1] ;
			dataset2D[3 * i + 2] = 0 ;
		}
		flann::Matrix<float> dataset = flann::Matrix<float>(dataset2D , m_num_of_cloud , 3) ;
		m_index_2D.reset(new FLANNIndex(dataset , ::flann::KDTreeSingleIndexParams(15))) ;
		m_index_2D->buildIndex() ;
	}
	else
	{
		std::cout<<"Set wrong search method"<<std::endl ;
		return false ;
	}
	return 1 ;
}

//计算点云邻域的中心点
int NearestSearch::computeCentroid3D(const std::vector<int> &indices , float &x , float &y , float &z)
{
	double sumx = 0;
	double sumy = 0;
	double sumz = 0;
    for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it)
	{
		sumx += m_dataset[*it * 3 + 0];
		sumy += m_dataset[*it * 3 + 1];
		sumz += m_dataset[*it * 3 + 2];
	}
	x = sumx / indices.size();
	y = sumy / indices.size();
	z = sumz / indices.size();
	return 1;
}

//计算特征值及特征向量
int NearestSearch::computeCovarianceEigen(new3s_PointXYZ &point , std::vector<float> &eigenvalues , std::vector<vecFloat> &eigenvectors)
{
	std::vector<float>dist;
	std::vector<int>indices;
	if (m_search_method == 0)
	{				
		nearestKSearch(point, m_search_number , indices , dist);
		/*std::cout<<indices.size()<<std::endl;*/
	}
	else if (m_search_method == 1)
	{
		nearestRSearch(point, m_search_radius , indices , dist);
		std::cout<<indices.size()<<std::endl;
	}
	else if (m_search_method == 2)
	{			
		std::vector<vecInt> radius_indices(1) ;
		std::vector<vecFloat> radius_dists(1) ;
		float *query = new float[3] ;
		query[0] = point.get_x() ;
		query[1] = point.get_y() ;
		query[2] = 0 ;
        flann::Matrix<float> queryset = flann::Matrix<float>(query , 1 , 3) ;
		int neighbors_in_radius = m_index_2D->radiusSearch(queryset , radius_indices , radius_dists , m_search_radius , flann::SearchParams(15)) ;		
		/*std::cout<<radius_indices[0].size()<<std::endl;*/
		for (std::vector<int>::iterator it = radius_indices[0].begin(); it != radius_indices[0].end(); ++it)
		{
			if ( abs(m_dataset[*it * 3 + 2] - point.get_z()) < m_search_rz)
				indices.push_back(*it) ;
		}
	}
	
	float cx;
	float cy;
	float cz;
	computeCentroid3D(indices , cx , cy , cz);
	Eigen::MatrixXf m (indices.size() , 3);
	for (std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it)
	{
		m(it-indices.begin() , 0) = m_dataset[*it * 3 + 0] - cx ;
		m(it-indices.begin() , 1) = m_dataset[*it * 3 + 1] - cy ;
		m(it-indices.begin() , 2) = m_dataset[*it * 3 + 2] - cz ;
	}
	Eigen::Matrix3f Cov ;
	Cov = m.transpose() * m ; 
	/*std::cout<<m<<std::endl;*/
	Eigen::EigenSolver<Eigen::Matrix3f> CovarianceMatrix(Cov);
	Eigen::Matrix3f D = CovarianceMatrix.pseudoEigenvalueMatrix();
	Eigen::Matrix3f V = CovarianceMatrix.pseudoEigenvectors();
	eigenvalues.push_back(D(0,0));
	eigenvalues.push_back(D(1,1));
	eigenvalues.push_back(D(2,2));
	for (int i = 0; i < 3; ++i)
	{
		std::vector<float>eigenvector ;
		for (int j = 0; j < 3; ++j)
		{
			eigenvector.push_back(V(i,j));
		}
		eigenvectors.push_back(eigenvector);
	}	
	return 1;
}

int NearestSearch::computeCovarianceEigen(const std::vector<int> &indices , std::vector<float> &eigenvalues , std::vector<std::vector<float> > &eigenvectors)
{
    float cx = 0 , cy = 0 , cz = 0 ;
    computeCentroid3D(indices , cx , cy , cz);
    Eigen::MatrixXf m (indices.size() , 3);
    for (size_t i = 0 ; i != indices.size() ; ++i)
    {
        m(i , 0) = m_dataset[i * 3 + 0] - cx ;
        m(i , 1) = m_dataset[i * 3 + 1] - cy ;
        m(i , 2) = m_dataset[i * 3 + 2] - cz ;
    }
    Eigen::Matrix3f Cov ;
    Cov = m.transpose() * m ;
    /*std::cout<<m<<std::endl;*/
    Eigen::EigenSolver<Eigen::Matrix3f> CovarianceMatrix(Cov);
    Eigen::Matrix3f D = CovarianceMatrix.pseudoEigenvalueMatrix();
    Eigen::Matrix3f V = CovarianceMatrix.pseudoEigenvectors();
    eigenvalues.push_back(D(0,0));
    eigenvalues.push_back(D(1,1));
    eigenvalues.push_back(D(2,2));
    for (int i = 0; i < 3; ++i)
    {
        std::vector<float>eigenvector ;
        for (int j = 0; j < 3; ++j)
        {
            eigenvector.push_back(V(i,j));
        }
        eigenvectors.push_back(eigenvector);
    }
    return 1;
}

//计算法向量
int NearestSearch::normalVectorEstimation(new3s_PointXYZ &point ,std::vector<float> &normalVector)
{
	std::vector<float>eigenvalues ;
    std::vector<std::vector<float> >eigenvectors ;
	computeCovarianceEigen(point , eigenvalues , eigenvectors);
	float minEigenValue = 10000 ;
	int mark = 0 ;
	for (int i = 0 ; i < 3 ; ++i)
	{
		if (eigenvalues[i] < minEigenValue)
		{
			minEigenValue = eigenvalues[i];
			mark = i ;
		}
	}
	normalVector = eigenvectors[mark] ;
	return 1;
}

int NearestSearch::VectorEstimation(const std::vector<int> &indices , std::vector<float> &vec)
{
    std::vector<float> eigenvalues ;
    std::vector<std::vector<float> > eigenvectors ;
    computeCovarianceEigen(indices , eigenvalues , eigenvectors);
    float maxEigenValue = INT_MIN ;
    int mark = 0 ;
    for (int i = 0 ; i < 3 ; ++i)
    {
        if (eigenvalues[i] > maxEigenValue)
        {
            maxEigenValue = eigenvalues[i];
            mark = i ;
        }
    }
    vec = eigenvectors[mark] ;
    return 1;
}

int NearestSearch::VectorEstimation(const std::vector<int> &indices, Eigen::Vector3d &vec)
{
    std::vector<float> eigenvalues ;
    std::vector<std::vector<float> > eigenvectors ;
    computeCovarianceEigen(indices , eigenvalues , eigenvectors);
    float maxEigenValue = INT_MIN ;
    int mark = 0 ;
    for (int i = 0 ; i < 3 ; ++i)
    {
        if (eigenvalues[i] > maxEigenValue)
        {
            maxEigenValue = eigenvalues[i];
            mark = i ;
        }
    }
    vec = Eigen::Vector3d(eigenvectors[mark][0] , eigenvectors[mark][1] , eigenvectors[mark][2]) ;
    return 1;
}
