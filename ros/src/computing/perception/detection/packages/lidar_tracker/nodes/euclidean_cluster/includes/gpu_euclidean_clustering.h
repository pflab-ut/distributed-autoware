#ifndef GPU_EUCLIDEAN_H_
#define GPU_EUCLIDEAN_H_

#include <iostream>
#include <vector>
#include <chrono>

class GpuEuclideanCluster {
public:
	typedef struct {
		int index_value;
		std::vector<int> points_in_cluster;
	} GClusterIndex;

	typedef struct {
		float *x;
		float *y;
		float *z;
		int size;
	} SamplePointListXYZ;

	double d_time_, m_time_, sub_time_;
	std::chrono::time_point<std::chrono::system_clock> d_start_, d_end_, m_start_, m_end_, sub_start_, sub_end_;

	GpuEuclideanCluster();

	void setInputPoints(float *x, float *y, float *z, int size);
	void setThreshold(double threshold);
	void setMinClusterPts(int min_cluster_pts);
	void setMaxClusterPts(int max_cluster_pts);
	void extractClustersOld();
	void extractClusters();
	void extractClusters2();
	std::vector<GClusterIndex> getOutput();

	SamplePointListXYZ generateSample();

	~GpuEuclideanCluster();

private:
	float *x_, *y_, *z_;
	int size_;
	double threshold_;
	int *cluster_indices_;
	int *cluster_indices_host_;
	int min_cluster_pts_;
	int max_cluster_pts_;
	int cluster_num_;

	void exclusiveScan(int *input, int ele_num, int *sum);
};

#endif
