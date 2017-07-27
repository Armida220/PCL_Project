#include "new3s_io.h"

const std::string DATASET_NAME("/Points") ;

new3s_io::new3s_io(void): m_inputfilename(NULL) , m_outputfilename(NULL)
{
}

new3s_io::~new3s_io(void)
{
	m_inputfilename = NULL ;
	m_outputfilename = NULL ;
}

// 读取las到vector
int new3s_io::lasFileRead(const char *inputlasfile , std::vector<new3s_PointXYZ> &points)
{
	LASreadOpener lasreadopener ;
	lasreadopener.set_file_name(inputlasfile) ;
	if (!lasreadopener.active())
	{
		std::cout<<"not active !\n" ;
		exit(-1) ;
	}
	LASreader *lasreader = lasreadopener.open() ;
	if (!lasreader)
	{
		std::cout<<"can not open file!\n" ;
		exit(-1) ;
	}

	int num = 0 ;
	while(lasreader->read_point())
	{
		points.push_back(new3s_PointXYZ(lasreader->point.get_x() , lasreader->point.get_y() , lasreader->point.get_z())) ;
		num ++ ;
	}
	lasreader->close();
	delete lasreader;
	return num ;
}

int new3s_io::lasFileRead(const char *inputlasfile , std::vector<new3s_PointXYZ> &points , int class_number , std::vector<int> &readIndices)
{
	LASreadOpener lasreadopener ;
	lasreadopener.set_file_name(inputlasfile) ;
	if (!lasreadopener.active())
	{
		std::cout<<"not active !\n" ;
		exit(-1) ;
	}
	LASreader *lasreader = lasreadopener.open() ;
	if (!lasreader)
	{
		std::cout<<"can not open file!\n" ;
		exit(-1) ;
	}

	int num = 0 ;
	while(lasreader->read_point())
	{
		if (lasreader->point.get_classification() == class_number)
		{
			points.push_back(new3s_PointXYZ(lasreader->point.get_x() , lasreader->point.get_y() , lasreader->point.get_z())) ;
			readIndices.push_back(num) ;
		}	
		num ++ ;
	}
	return num ;
}

int new3s_io::lasFileRead(const char *inputlasfile, std::vector<new3s_PointXYZRGB> &points)
{
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name(inputlasfile);
	if (!lasreadopener.active())
	{
		std::cout << "not active !\n";
		exit(-1);
	}
	LASreader *lasreader = lasreadopener.open();
	if (!lasreader)
	{
		std::cout << "can not open file!\n";
		exit(-1);
	}
	double x = 0, y = 0, z = 0;
	size_t R = 0, G = 0, B = 0;
	while (lasreader->read_point())
	{
		x = lasreader->point.get_x();
		y = lasreader->point.get_y();
		z = lasreader->point.get_z();
		R = (int)(((float)lasreader->point.get_R()/65535) * 255 + 0.5);
		G = (int)(((float)lasreader->point.get_G()/65535) * 255 + 0.5);
		B = (int)(((float)lasreader->point.get_B()/65535) * 255 + 0.5);
		points.push_back(new3s_PointXYZRGB(x, y, z, R, G, B));
	}

	int num = 0;
}

// 读取h5到vector
// int new3s_io::H5FileRead(char *inputH5file , char *inputFormat , std::vector<new3s_PointXYZ> &points)
// {
// 	H5::H5File file(inputH5file , H5F_ACC_RDONLY) ;
// 	H5::DataSet dataset = file.openDataSet(DATASET_NAME) ;
// 	H5::DataSpace dataspace = dataset.getSpace() ;//获取数据集的数据空间
// 	int rank = dataspace.getSimpleExtentNdims() ;//获取数据空间的维度
// 	hsize_t dims_out[2] ;
// 	rank = dataspace.getSimpleExtentDims(dims_out , NULL) ;
// 
// 	std::string format_i = inputFormat ;
// 	int num_of_cloud = dims_out[0] ;
// 	int num_of_point_pro = format_i.size() ;
// 	if (num_of_point_pro != dims_out[1])
// 	{
// 		std::cout<<"h5 file format is wrong , please check it again!  "<<std::endl ;
// 		exit(-1) ;
// 	}
// 
// 	double *data = new double[dims_out[0] * dims_out[1]] ;
// 	dataset.read(data , H5::PredType::NATIVE_DOUBLE) ;
// 
// 	points.resize(num_of_cloud) ;
// 
// 	for (int i = 0 ; i < num_of_cloud ; ++i)
// 	{
// 		for (int j = 0;j < format_i.size() ; ++j)
// 		{
// 			switch (format_i[j])
// 			{
// 			case 'X':
// 				points[i].set_x(data[i * num_of_point_pro + j]) ;
// 				break;
// 			case 'Y':
// 				points[i].set_y(data[i * num_of_point_pro + j]) ;
// 				break;
// 			case 'Z':
// 				points[i].set_z(data[i * num_of_point_pro + j]) ;
// 				break;
// 			}
// 		}
// 	}
// 	return num_of_cloud ;
// }

// int new3s_io::H5FileRead(char *inputH5file , char *inputFormat , std::vector<new3s_PointXYZ> &points , int class_number , std::vector<int> &readIndices)
// {
// 	H5::H5File file(inputH5file , H5F_ACC_RDONLY) ;
// 	H5::DataSet dataset = file.openDataSet(DATASET_NAME) ;
// 	H5::DataSpace dataspace = dataset.getSpace() ;//获取数据集的数据空间
// 	int rank = dataspace.getSimpleExtentNdims() ;//获取数据空间的维度
// 	hsize_t dims_out[2] ;
// 	rank = dataspace.getSimpleExtentDims(dims_out , NULL) ;
// 
// 	std::string format_i = inputFormat ;
// 	hsize_t num_of_cloud = dims_out[0] ;
// 	size_t num_of_point_pro = format_i.size() ;
// 	if (num_of_point_pro != dims_out[1])
// 	{
// 		std::cout<<"h5 file format is wrong , please check it again!  "<<std::endl ;
// 		exit(-1) ;
// 	}
// 
// 	double *data = new double[dims_out[0] * dims_out[1]] ;
// 	dataset.read(data , H5::PredType::NATIVE_DOUBLE) ;
// 
// 	for (int i = 0 ; i < num_of_cloud ; ++i)
// 	{
// 		new3s_PointXYZ point3d ;
// 		int point_C ;
// 		for (int j=0;j < format_i.size() ; ++j)
// 		{			
// 			switch (format_i[j])
// 			{
// 			case 'X':
// 				point3d.set_x(data[i * num_of_point_pro + j]) ;
// 				break;
// 			case 'Y':
// 				point3d.set_y(data[i * num_of_point_pro + j]) ;
// 				break;
// 			case 'Z':
// 				point3d.set_z(data[i * num_of_point_pro + j]) ;
// 				break;
// 			case 'C':
// 				point_C = data[i * num_of_point_pro + j] ;
// 				break;
// 			}			
// 		}
// 		if (point_C == class_number)
// 		{
// 			points.push_back(point3d) ;
// 			readIndices.push_back(i) ;
// 		}
// 	}
// 	return num_of_cloud ;
// }


// 输入las输出las
// int new3s_io::updateClustersLasToLas(std::vector<int> &point_clusters)
// {
// 	LASreadOpener lasreadopener;
// 	lasreadopener.set_file_name(m_inputfilename);
// 	LASreader *lasreader = lasreadopener.open();
// 
// 	if (lasreader->header.number_of_point_records != point_clusters.size())
// 	{
// 		std::cout<<"ERROR: incorrect size of input point_clusters !"<<std::endl ;
// 		exit(-1) ;
// 	}
// 	LASwriteOpener laswriteopener;
// 	laswriteopener.set_file_name(m_outputfilename);
// 	LASheader new_header = lasreader->header;
// 
// 	new_header.point_data_format = 2;
// 	new_header.point_data_record_length = 28;
// 
// 	LASpoint point_temp;
// 	point_temp.init(&new_header, new_header.point_data_format, new_header.point_data_record_length, &new_header);
// 	LASwriter *laswrite = laswriteopener.open(&new_header);
// 
// 	int m = 0 ;
// 	int n = 0 ;
// 	while(lasreader->read_point())
// 	{	
// 		if (point_clusters[m] != 0)
// 		{
// 			point_temp = lasreader->point ;
// 			point_temp.set_point_source_ID(point_clusters[m]) ;
// 			laswrite->write_point(&point_temp) ;	
// 		}			
// 		m++;
// 	}
// 	laswrite->update_header(&new_header, TRUE);
// 	laswrite->close();
// 	delete laswrite;
// 	lasreader->close();
// 	return 0 ; 
// }

// 输入las输出h5
// int new3s_io::updateClustersLasToH5(std::vector<int> &point_clusters , char *outputFormat)
// {
// 	LASreadOpener lasreadopener;
// 	lasreadopener.set_file_name(m_inputfilename);
// 	if (!lasreadopener.active())
// 	{
// 		std::cout<<"not active !\n" ;
// 		exit(-1) ;
// 	}
// 
// 	LASreader *lasreader = lasreadopener.open();
// 	if (!lasreader)
// 	{
// 		std::cout<<"can not open file!\n" ;
// 		exit(-1) ;
// 	}
// 
// 	if (lasreader->header.number_of_point_records != point_clusters.size())
// 	{
// 		std::cout<<"ERROR: incorrect size of input point_clusters !"<<std::endl ;
// 		exit(-1) ;
// 	}
// 
// 	int num_of_cloud = lasreader->header.number_of_point_records ;
// 	std::string format_o = outputFormat ;
// 	int num_of_point_pro = format_o.size() ;
// 	if (num_of_point_pro > 18 )
// 	{
// 		std::cout<<"ERROR: format is not right, we only use 14 now yet,please check it again!  "<<std::endl;
// 		exit(-1);
// 	}
// 
// 	double *data = new double[num_of_cloud * num_of_point_pro] ;
// 	for (int i = 0 ; i < num_of_cloud ; ++i)
// 	{
// 		for (int j = 0 ; j < num_of_point_pro ; ++j)
// 		{
// 			data[num_of_point_pro * i + j] = 0 ;
// 		}
// 	}
// 
// 
// 	int i = 0;
// 	while(lasreader->read_point())
// 	{
// 		for (int j=0;j<num_of_point_pro;j++)
// 		{
// 			switch (format_o[j])
// 			{
// 			case 'X':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_x() ;
// 				break;
// 			case 'Y':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_y() ;
// 				break;
// 			case 'Z':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_z() ;
// 				break;
// 			case 'r':
// 				data[num_of_point_pro * i + j] =(double) lasreader->point.get_R() ;
// 				break;
// 			case 'g':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_G() ;
// 				break;
// 			case 'b':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_B() ;
// 				break;
// 			case 'T':
// 				data[num_of_point_pro * i + j] =(double) lasreader->point.get_gps_time() ;
// 				break;
// 			case 'R':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_return_number() ;
// 				break;
// 			case 'K':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_scan_angle_rank() ;
// 				break;
// 			case 'P':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_number_of_returns() ;
// 				break;
// 			case 'C':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_classification() ;
// 				break;
// 			case 'I':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_intensity() ;
// 				break;
// 			case 'D':
// 				data[num_of_point_pro * i + j] =(double) lasreader->point.get_scan_direction_flag() ;
// 				break;
// 			case 'L':
// 				data[num_of_point_pro * i + j] = (double)lasreader->point.get_point_source_ID() ;
// 				break;
// 			case 'c':
// 				data[num_of_point_pro * i + j] = (double)point_clusters[i] ;
// 				break;
// 			}
// 		}
// 		i++;
// 	}
// 
// 	lasreader->close();
// 
// 	H5::H5File file(m_outputfilename , H5F_ACC_TRUNC) ;
// 	hsize_t dimsf[2] ;
// 	dimsf[0] = num_of_cloud ;
// 	dimsf[1] = num_of_point_pro ;
// 	H5::DataSpace dataspace(2 , dimsf , NULL) ;
// 
// 	H5::IntType datatype(H5::PredType::NATIVE_DOUBLE) ;
// 	datatype.setOrder(H5T_ORDER_LE) ;
// 
// 	H5::DataSet dataset = file.createDataSet(DATASET_NAME , datatype , dataspace) ;
// 	dataset.write(data , H5::PredType::NATIVE_DOUBLE) ;
// 
// 	return 0;
// }

// 输入h5输出las
// int new3s_io::updateClustersH5ToLas(std::vector<int> &point_clusters , char *inputFormat)
// {
// 	H5::H5File file(m_inputfilename , H5F_ACC_RDONLY) ;
// 	H5::DataSet dataset = file.openDataSet(DATASET_NAME) ;
// 
// 	H5::DataSpace dataspace = dataset.getSpace() ;//获取数据集的数据空间
// 	int rank = dataspace.getSimpleExtentNdims() ;//获取数据空间的维度
// 
// 	hsize_t dims_out[2] ;
// 	rank = dataspace.getSimpleExtentDims(dims_out , NULL) ;
// 
// 	std::string format_i = inputFormat ;
// 	int num_of_point_pro = format_i.size() ;
// 	if (num_of_point_pro != dims_out[1])
// 	{
// 		std::cout<<"h5 file format is wrong , please check it again!  "<<std::endl ;
// 		exit(-1) ;
// 	}
// 
// 	double *data = new double[dims_out[0] * dims_out[1]] ;
// 	dataset.read(data , H5::PredType::NATIVE_DOUBLE) ;
// 
// 	std::cout<<"all have readed here in ram"<<std::endl ;
// 
// 	//////////////////////////////////////////////////////////////////////////// 写出las文件
// 	LASwriteOpener laswriteropener ;
// 	laswriteropener.set_file_name(m_outputfilename) ;
// 
// 	LASheader lasheader ;
// 	lasheader.x_scale_factor = 0.001f ;
// 	lasheader.y_scale_factor = 0.1f ;
// 	lasheader.z_scale_factor = 0.001f ;
// 	lasheader.x_offset = 0;
// 	lasheader.y_offset = 0;
// 	lasheader.z_offset = 0;
// 	lasheader.point_data_format = 2;
// 	lasheader.point_data_record_length = 28;
// 
// 	LASpoint laspoint ;
// 	laspoint.init(&lasheader , lasheader.point_data_format , lasheader.point_data_record_length , 0) ;
// 	LASwriter *laswriter = laswriteropener.open(&lasheader) ;
// 	if (laswriter == 0)
// 	{
// 		std::cout<<"failture !\n" <<std::endl ;
// 		return 0 ;
// 	}
// 	int num_of_cloud = dims_out[0] ;
// 
// 	for (int i = 0 ; i < num_of_cloud ; ++i)
// 	{
// 		if (point_clusters[i] != 0)
// 		{
// 			for (int j=0;j < format_i.size();j++)
// 			{
// 				switch (format_i[j])
// 				{
// 				case 'X':
// 					laspoint.set_x(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'Y':
// 					laspoint.set_y(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'Z':
// 					laspoint.set_z(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'r':
// 					laspoint.set_R(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'g':
// 					laspoint.set_G(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'b':
// 					laspoint.set_B(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'T':
// 					laspoint.set_gps_time(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'R':
// 					laspoint.set_return_number(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'K':
// 					laspoint.set_scan_angle_rank(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'P':
// 					laspoint.set_number_of_returns(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'C':
// 					laspoint.set_classification(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'I':
// 					laspoint.set_intensity(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'D':
// 					laspoint.set_scan_direction_flag(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'L':
// 					laspoint.set_point_source_ID(data[i * num_of_point_pro + j]) ;
// 					break;
// 				case 'A':
// 					laspoint.set_point_source_ID(point_clusters[i]) ;
// 					break;
// 				case 'c':
// 					laspoint.set_point_source_ID(point_clusters[i]) ;
// 					break;
// 				}
// 			}
// 			laswriter->write_point(&laspoint) ;
// 			laswriter->update_inventory(&laspoint) ;
// 		}				
// 	}
// 	laswriter->update_header(&lasheader , TRUE) ;
// 	laswriter->close() ;
// 
// 	delete laswriter ;
// 	return 0;
// }

// 输入h5输出h5
// int new3s_io::updateClustersH5ToH5(std::vector<int> &point_clusters , char *inputFormat , char *outputFormat)
// {
// 	H5::H5File filein(m_inputfilename , H5F_ACC_RDONLY) ;
// 	H5::DataSet datasetin = filein.openDataSet(DATASET_NAME) ;
// 
// 	H5::DataSpace dataspacein = datasetin.getSpace() ;//获取数据集的数据空间
// 	int rank = dataspacein.getSimpleExtentNdims() ;//获取数据空间的维度
// 
// 	hsize_t dims_out[2] ;
// 	rank = dataspacein.getSimpleExtentDims(dims_out , NULL) ;
// 
// 	std::string format_i = inputFormat ;
// 	std::string format_o = outputFormat ;
// 	size_t num_of_cloud = dims_out[0] ;
// 	size_t num_of_point_pro = format_i.size() ;
// 	if (num_of_point_pro != dims_out[1])
// 	{
// 		std::cout<<"h5 file format is wrong , please check it again!  "<<std::endl ;
// 		exit(-1) ;
// 	}
// 
// 	double *data = new double[dims_out[0] * dims_out[1]] ;
// 	datasetin.read(data , H5::PredType::NATIVE_DOUBLE) ;
// 
// 	std::cout<<"all have readed here in ram"<<std::endl ;
// 
// 	///////////////////////////////////////////////////////////// 存储点的中间文件
// 
// 	std::vector<new3s_PointXYZ> points3d ;
// 	std::vector<double> point_r ;
// 	std::vector<double> point_g ;
// 	std::vector<double> point_b ;
// 	std::vector<double> point_T ;
// 	std::vector<double> point_R ;
// 	std::vector<double> point_K ;
// 	std::vector<double> point_P ;
// 	std::vector<double> point_C ;
// 	std::vector<double> point_I ;
// 	std::vector<double> point_D ;
// 	std::vector<double> point_L ;
// 	std::vector<double> point_S ;
// 	std::vector<double> point_G ;
// 	std::vector<double> point_A ;
// 	std::vector<double> point_c ;
// 
// 
// 	for (int j = 0; j < format_o.size(); ++j)
// 	{
// 		switch (format_o[j])
// 		{
// 		case 'X' :
// 			points3d.resize(num_of_cloud) ;
// 			break ;
// 		case 'r':
// 			point_r.resize(num_of_cloud) ;
// 			break;
// 		case 'g':
// 			point_g.resize(num_of_cloud) ;
// 			break;
// 		case 'b':
// 			point_b.resize(num_of_cloud) ;
// 			break;
// 		case 'T':
// 			point_T.resize(num_of_cloud) ;
// 			break;
// 		case 'R':
// 			point_R.resize(num_of_cloud) ;
// 			break;
// 		case 'K':
// 			point_K.resize(num_of_cloud) ;
// 			break;
// 		case 'P':
// 			point_P.resize(num_of_cloud) ;
// 			break;
// 		case 'C':
// 			point_C.resize(num_of_cloud) ;
// 			break;
// 		case 'I':
// 			point_I.resize(num_of_cloud) ;
// 			break;
// 		case 'D':
// 			point_D.resize(num_of_cloud) ;
// 			break;
// 		case 'L':
// 			point_L.resize(num_of_cloud) ;
// 			break;
// 		case 'S':
// 			point_S.resize(num_of_cloud) ;
// 			break;
// 		case 'G':
// 			point_G.resize(num_of_cloud) ;
// 			break;
// 		case 'A':
// 			point_A.resize(num_of_cloud) ;
// 			break;
// 		case 'c':
// 			point_c.resize(num_of_cloud) ;
// 			break;
// 		}
// 	}
// 
// 
// 
// 	for (int i = 0 ; i < num_of_cloud ; ++i)
// 	{
// 		for (int j = 0; j < format_i.size(); ++j)
// 		{
// 			switch (format_i[j])
// 			{
// 			case 'X':
// 				if (points3d.size() != 0)
// 				{
// 					points3d[i].set_x(data[i * num_of_point_pro + j]) ;
// 				}				
// 				break;
// 			case 'Y':
// 				if (points3d.size() != 0)
// 				{
// 					points3d[i].set_y(data[i * num_of_point_pro + j]) ;
// 				}		
// 				break;
// 			case 'Z':
// 				if (points3d.size() != 0)
// 				{
// 					points3d[i].set_z(data[i * num_of_point_pro + j]) ;
// 				}	
// 				break;
// 			case 'r':
// 				if (point_r.size() != 0)
// 				{
// 					point_r[i] = data[i * num_of_point_pro + j] ;
// 				}	
// 				break;
// 			case 'g':
// 				if (point_g.size() != 0)
// 				{
// 					point_g[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'b':
// 				if (point_b.size() != 0)
// 				{
// 					point_b[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'T':
// 				if (point_T.size() != 0)
// 				{
// 					point_T[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'R':
// 				if (point_R.size() != 0)
// 				{
// 					point_R[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'K':
// 				if (point_K.size() != 0)
// 				{
// 					point_K[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'P':
// 				if (point_P.size() != 0)
// 				{
// 					point_P[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'C':
// 				if (point_C.size() != 0)
// 				{
// 					point_C[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'I':
// 				if (point_I.size() != 0)
// 				{
// 					point_I[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'D':
// 				if (point_D.size() != 0)
// 				{
// 					point_D[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'L':
// 				if (point_L.size() != 0)
// 				{
// 					point_L[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'S':
// 				if (point_S.size() != 0)
// 				{
// 					point_S[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'G':
// 				if (point_G.size() != 0)
// 				{
// 					point_G[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'A':
// 				if (point_A.size() != 0)
// 				{
// 					point_A[i] = data[i * num_of_point_pro + j] ;
// 				}
// 				break;
// 			case 'c':
// 				if (point_c.size() != 0)
// 				{
// 					point_c[i] = point_clusters[i] ;
// 				}
// 				break;
// 			}
// 		}
// 	}
// 
// 	//////////////////////////////////////////////////////////////////////////// 写出h5文件
// 	size_t new_num_of_point_pro = format_o.size() ;
// 	double *newData = new double[num_of_cloud * new_num_of_point_pro] ;
// 	for (int i = 0 ; i < num_of_cloud ; ++i)
// 	{
// 		for (int j = 0 ; j < new_num_of_point_pro ; ++j)
// 		{
// 			newData[num_of_point_pro * i + j] = 0 ;
// 		}
// 	}
// 
// 	for (int i = 0 ; i != num_of_cloud ; ++i)
// 	{
// 		for (int j = 0;j != new_num_of_point_pro ; ++j)
// 		{
// 			switch (format_o[j])
// 			{
// 			case 'X':
// 				newData[new_num_of_point_pro * i + j] = (double) points3d[i].get_x() ;
// 				break;
// 			case 'Y':
// 				newData[new_num_of_point_pro * i + j] = (double) points3d[i].get_y() ;
// 				break;
// 			case 'Z':
// 				newData[new_num_of_point_pro * i + j] = (double) points3d[i].get_z() ;
// 				break;
// 			case 'r':
// 				newData[new_num_of_point_pro * i + j] = (double) point_r[i] ;
// 				break;
// 			case 'g':
// 				newData[new_num_of_point_pro * i + j] = (double) point_g[i] ;
// 				break;
// 			case 'b':
// 				newData[new_num_of_point_pro * i + j] = (double) point_b[i] ;
// 				break;
// 			case 'T':
// 				newData[new_num_of_point_pro * i + j] = (double) point_T[i] ;
// 				break;
// 			case 'R':
// 				newData[new_num_of_point_pro * i + j] = (double) point_R[i] ;
// 				break;
// 			case 'K':
// 				newData[new_num_of_point_pro * i + j] = (double) point_K[i] ;
// 				break;
// 			case 'P':
// 				newData[new_num_of_point_pro * i + j] = (double) point_P[i] ;
// 				break;
// 			case 'C':
// 				newData[new_num_of_point_pro * i + j] = (double) point_C[i] ;
// 				break;
// 			case 'I':
// 				newData[new_num_of_point_pro * i + j] = (double) point_I[i] ;
// 				break;
// 			case 'D':
// 				newData[new_num_of_point_pro * i + j] = (double) point_D[i] ;
// 				break;
// 			case 'L':
// 				newData[new_num_of_point_pro * i + j] = (double) point_L[i] ;
// 				break;
// 			case 'S':
// 				newData[new_num_of_point_pro * i + j] = (double) point_A[i] ;
// 				break;
// 			case 'G':
// 				newData[new_num_of_point_pro * i + j] = (double) point_G[i] ;
// 				break;
// 			case 'A':
// 				newData[new_num_of_point_pro * i + j] = (double) point_A[i] ;
// 				break;
// 			case 'c':
// 				newData[new_num_of_point_pro * i + j] = (double) point_c[i] ;
// 				break;
// 			}
// 		}
// 	}
// 
// 	H5::H5File fileout(m_outputfilename , H5F_ACC_TRUNC) ;
// 	hsize_t dimsf[2] ;
// 	dimsf[0] = num_of_cloud ;
// 	dimsf[1] = new_num_of_point_pro ;
// 	H5::DataSpace dataspaceout(2 , dimsf , NULL) ;
// 
// 	H5::IntType datatype(H5::PredType::NATIVE_DOUBLE) ;
// 	datatype.setOrder(H5T_ORDER_LE) ;
// 
// 	H5::DataSet datasetout = fileout.createDataSet(DATASET_NAME , datatype , dataspaceout) ;
// 	datasetout.write(newData , H5::PredType::NATIVE_DOUBLE) ;
// 
// 	return 0;
// }

// 封装
// int new3s_io::updateClusters(std::vector<int> &point_clusters , char *inputFormat , char *outputFormat)
// {
// 	char *inext = strchr(m_inputfilename ,'.') ;
// 	char *outext = strchr(m_outputfilename ,'.') ;
// 
// 	if (strcmp(outext,".las") == 0 && strcmp(inext,".las") == 0)
// 	{
// 		updateClustersLasToLas(point_clusters) ;
// 	}
// 	else if (strcmp(outext,".h5") == 0 && strcmp(inext,".las") == 0)
// 	{
// 		updateClustersLasToH5(point_clusters , outputFormat) ;
// 	}
// 	else if (strcmp(outext,".las") == 0 && strcmp(inext,".h5") == 0)
// 	{
// 		updateClustersH5ToLas(point_clusters , inputFormat) ;
// 	}
// 	else if (strcmp(outext,".h5") == 0 && strcmp(inext,".h5") == 0)
// 	{
// 		updateClustersH5ToH5(point_clusters , inputFormat , outputFormat) ;
// 	}
// 	else
// 	{
// 		std::cout<<"Incorrect output data format !"<<std::endl ;
// 		exit(-1) ;
// 	}
// 
// 	return 0 ;
// }

int new3s_io::writeVectoLas(const std::vector<new3s_PointXYZ> &cloud , const int &R , const int &G , const int &B , char *lasname)
{
	if (cloud.empty())
	{
		std::cout<<"点云为空！\n" ;
		return 0 ;
	}

	size_t num = cloud.size() ;
	LASwriteOpener laswriteopener ;
	laswriteopener.set_file_name(lasname) ;
	LASheader lasheader ;

	lasheader.x_scale_factor = 0.1;
	lasheader.y_scale_factor = 0.01;
	lasheader.z_scale_factor = 0.001;
	lasheader.x_offset = 1000.0;
	lasheader.y_offset = 2000.0;
	lasheader.z_offset = 0.0;
	lasheader.point_data_format = 2;
	lasheader.point_data_record_length = 28;
	LASpoint pt ;
	pt.init(&lasheader , lasheader.point_data_format , lasheader.point_data_record_length , 0) ;

	LASwriter *laswriter = laswriteopener.open(&lasheader) ;
	if (!laswriter)
	{
		std::cout<<"failture!\n" ;
		return 0 ;
	}

	//颜色255 -> 65535
	double val1 = (double)(65535/255) ;
	U16 R16 = (U16)(val1 * R) ;
	U16 G16 = (U16)(val1 * G) ;
	U16 B16 = (U16)(val1 * B) ;
	for (int i = 0 ; i < num ; i++)
	{
		pt.set_x(cloud[i].get_x()) ;
		pt.set_y(cloud[i].get_y()) ;
		pt.set_z(cloud[i].get_z()) ;
		pt.set_R(R16) ;
		pt.set_G(G16) ;
		pt.set_B(B16) ;
		laswriter->write_point(&pt) ;
		laswriter->update_inventory(&pt) ;
	}
	laswriter->update_header(&lasheader , TRUE) ;
	laswriter->close() ;

	delete laswriter ;
	return 1 ;
}