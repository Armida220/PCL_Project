#include <pcl/console/parse.h>  
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/octree/octree_pointcloud_adjacency_container.h>
#include "new_supervoxel_clustering.h" 
#include "new_supervoxel_clustering.hpp" 
#include <vtkPolyLine.h>    //  �����Ҫ��ӣ�����ᱨ��  

// Types  
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;//L��Label
typedef pcl::PointCloud<PointLT> PointLCloudT;

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,//��������
	PointCloudT &adjacent_supervoxel_centers, //�����ڽ���������
	std::string supervoxel_name,        //������
	boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);


int
main(int argc, char ** argv)//argv�ַ������飬�������ָ������ַ���������ָ�����飬ÿһ��Ԫ��ָ��һ������
{



	PointCloudT::Ptr cloud = boost::make_shared <PointCloudT>();
	pcl::console::print_highlight("Loading point cloud...\n");
	if (pcl::io::loadPCDFile<PointT>("test.pcd", *cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		return (1);
	}

	/************************************************************************/
	/* �趨�ᾧ����                                                         */
	/************************************************************************/
	//	bool use_transform = !pcl::console::find_switch(argc, argv, "--NT");

	float voxel_resolution = 0.4f;//���Ӿ��룬�����ǰ˲����ֱ���
	//	bool voxel_res_specified = pcl::console::find_switch(argc, argv, "-v");
	//	if (voxel_res_specified)
	//		pcl::console::parse(argc, argv, "-v", voxel_resolution);

	float seed_resolution = 6.5f;//ָ�����˾���
	//	bool seed_res_specified = pcl::console::find_switch(argc, argv, "-s");
	//	if (seed_res_specified)
	//		pcl::console::parse(argc, argv, "-s", seed_resolution);

	float color_importance = 0.4f;//��ɫֵ
	//	if (pcl::console::find_switch(argc, argv, "-c"))
	//		pcl::console::parse(argc, argv, "-c", color_importance);

	float spatial_importance = 0.2f;
	//	if (pcl::console::find_switch(argc, argv, "-z"))
	//		pcl::console::parse(argc, argv, "-z", spatial_importance);

	float normal_importance = 0.4f;//������
	//	if (pcl::console::find_switch(argc, argv, "-n"))
	//		pcl::console::parse(argc, argv, "-n", normal_importance);

	//////////////////////////////  //////////////////////////////  
	////// This is how to use supervoxels  
	//////////////////////////////  //////////////////////////////  

	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);

	//����ᾧ�ָ����������һ��ӳ���
	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

	pcl::console::print_highlight("Extracting supervoxels!\n");
	super.extract(supervoxel_clusters);//����ָ����������һ��ӳ���
	pcl::console::print_info("Found %d supervoxels\n", supervoxel_clusters.size());

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();//��þ�������
	viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");



	/*********************************************************************/
	/* ���������ĵ�����д��������� ���ھ���                                 */
	/************************************************************************/
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud < pcl::PointXYZ>);
	// 	centroid_cloud->width = supervoxel_clusters.size();
	// 	centroid_cloud->height = 1;
	// 	centroid_cloud->is_dense = false;
	// 	centroid_cloud->points.resize(supervoxel_clusters.size());
	// 	std::cout << supervoxel_clusters.at(504)->centroid_.x << endl;
	// 	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > ::iterator centroid_itr = supervoxel_clusters.begin();
	// 	size_t num_i=0;
	// 	for (; centroid_itr != supervoxel_clusters.end(); ++centroid_itr)
	// 	{
	// 	//	pcl::Supervoxel<PointT>::Ptr centroid_supervoxel = supervoxel_clusters.at(centroid_itr->second);//�ڽ�����
	// 		centroid_cloud->points[num_i].x = supervoxel_clusters.at(centroid_itr->first)->centroid_.x;
	// 		centroid_cloud->points[num_i].y = supervoxel_clusters.at(centroid_itr->first)->centroid_.y;
	// 		centroid_cloud->points[num_i].z = supervoxel_clusters.at(centroid_itr->first)->centroid_.z;
	// 		num_i++;
	// 	}
	// 	pcl::PCDWriter writer;
	// 	writer.write("C:\\Users\\Administrator\\Desktop\\centroid123.pcd", *centroid_cloud, false);



	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");//��ָ����viewport������id���Ƶ���Ⱦ����
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

	PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();//��ȡ����
	//PointCloudT::Ptr colored_voxel_cloud = super.getColoredVoxelCloud();///��þ��������ɫ
	viewer->addPointCloud(labeled_voxel_cloud, "colored_voxels");
	//	viewer->addPointCloud(colored_voxel_cloud, "colored voxels");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "colored voxels");


	//PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
	////We have this disabled so graph is easy to see, uncomment to see supervoxel normals  
	//   //viewer->addPointCloudNormals<pcl::Po intNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");  
	//

	////ִ���������̺󣬻Ὣ����ӳ���һϵ���������������ָ����������ָ�롣����ͨ��getter�������Ѿ����йص���Ϣ�ϳ������ϳ������ǵ�

	////�����ڵľ�������������������ʾ
	//pcl::console::print_highlight("Getting supervoxel adjacency\n");
	//std::multimap<uint32_t, uint32_t> supervoxel_adjacency;//���ڵľ�������-Ӧ����һ��2*2�ľ���
	//super.getSupervoxelAdjacency(supervoxel_adjacency);//��ȡ���ھ�������
	////To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap  
	//std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
	//for (; label_itr != supervoxel_adjacency.end();)
	//{
	//	//First get the label  
	//	uint32_t supervoxel_label = label_itr->first;//��ȡָ���Ӧ����
	//	//Now get the supervoxel corresponding to the label  
	//	pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);//��ȡ��Ӧ��ŵĶ�Ӧ����ָ�룬ָ��һ������ĵ�ַ

	//	//Now we need to iterate through the adjacent supervoxels and make a point cloud of them  
	//	PointCloudT adjacent_supervoxel_centers;//�ڽ���������
	//	std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
	//	for (; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
	//	{
	//		pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);//�ڽ�����
	//		adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);//���ڽ��������ķ���adjacent_supervoxel_centers
	//	}
	//	//Now we make a name for this polygon  
	//	std::stringstream ss;
	//	ss << "C:\\Users\\Administrator\\Desktop\\supervoxel_" << supervoxel_label;
	//	//This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given  
	//	addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), viewer);
	//	//Move iterator forward to next label  
	//	label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
	//}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
	std::cin.get();
	std::cin.get();

	return (0);
}

void
addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
	PointCloudT &adjacent_supervoxel_centers,
	std::string supervoxel_name,
	boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

	//Iterate through all adjacent points, and add a center point to adjacent point pair  
	PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin();
	for (; adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
	{
		points->InsertNextPoint(supervoxel_center.data);
		points->InsertNextPoint(adjacent_itr->data);
	}
	// Create a polydata to store everything in  
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	// Add the points to the dataset  
	polyData->SetPoints(points);
	polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
	for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
		polyLine->GetPointIds()->SetId(i, i);
	cells->InsertNextCell(polyLine);
	// Add the lines to the dataset  
	polyData->SetLines(cells);
	viewer->addModelFromPolyData(polyData, supervoxel_name);
}