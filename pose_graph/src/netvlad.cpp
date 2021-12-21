

#include "netvlad.h"

NetVLAD::NetVLAD(){
}

void NetVLAD::query(const std::vector<float>& query, DBoW2::QueryResults& query_results,
                    const int max_results, const int max_id) const {
  // TODO: check if this is slow and maybe use a heap, since we have to significantly reduce the number of results while checking *all* images
  query_results.clear();
if (max_id > 0){
  query_results.reserve(database_.size());
for(auto it = database_.begin(); it != database_.end(); it++) {
  if(it->first <= max_id) {
    // Following example of DBoW2, we throw away all ids unless < max_id
    query_results.emplace_back(it->first, score(query, it->second));
  } else {
      break;
  }
  
}
std::sort(query_results.rbegin(), query_results.rend()); // Sort query_results in reverse so it ends up descending
// printf("***ID22*** %d \n", query_results.size());

if(query_results.size() > max_results) {
  query_results.resize(max_results);
}
}
  
}
void NetVLAD::add(const std::vector<float>& rep, const unsigned int id) {
  database_.insert(database_.end(), std::make_pair(id, rep));
}
float getMold(const std::vector<float>& vec){   //求向量的模长
        int n = vec.size();
        float sum = 0.0;
        for (int i = 0; i<n; ++i)
            sum += vec[i] * vec[i];
        return sqrt(sum);
    }
double NetVLAD::score(const std::vector<float>& d1, const std::vector<float>& d2) const {
 // float result = rep1.transpose() * rep2;
  int n = d1.size();
  float tmp = 0.0;  //内积
  for (int i = 0; i<n; ++i)
          tmp += d1[i]* d2[i];
  return tmp / (getMold(d1)*getMold(d2));
	// return result;
  // Can assume that tensors are normalized, so Euclidean norm^2 is in [0, 2], with 0 best.
  // This score was designed to be interpreted via norm, so we return 1 - norm^2 / 2 to
  // follow convention of score in [0, 1] with 1 best

}
