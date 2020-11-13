# include <iostream>
# include <fstream>
# include <malloc.h>

# include <opencv2/core.hpp>
# include <opencv2/surface_matching.hpp>
# include <opencv2/surface_matching/ppf_helpers.hpp>
# include <opencv2/surface_matching/t_hash_int.hpp>

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

string filename = "/home/xxwang/Workspaces/YOLO_PPF_Pose_Estimation/src/detector_bluemoon_bottle.xml";

int default_hashtable_size = 100000;
hashtable_int *_hash_table = hashtableCreate(default_hashtable_size, NULL);
THash *_hash_nodes = (THash *)calloc(default_hashtable_size, sizeof(THash));

void LoadHashTable(const FileNode &fn)
{
    int hash_table_size;
    fn["hash_table_size"] >> hash_table_size;   

    hashtableResize(_hash_table, hash_table_size);
    realloc(_hash_nodes, hash_table_size);

    FileNode fn_nodes = fn["hash_table_nodes"];

    uint counter = 0;
    int id, i, ppf_ind;
    FileNode item;
    THash *thash_item;
    std::cout << "Hashtable initialized, starting Deserialization" << std::endl;
    std::cout << "length " << fn_nodes.end()-fn_nodes.begin() << std::endl;
    size_t size_fn_nodes = fn_nodes.size();
    
    for (FileNodeIterator it = fn_nodes.begin(); it != fn_nodes.end(); it++)
    {
        std::cout << "\r[Deserialization] Processing: " << (counter+1) << "/" << hash_table_size;
        item = *it;
        // item = fn_nodes[it];

        item["id"] >> id;
        item["i"] >> i;
        item["ppfInd"] >> ppf_ind;

        thash_item = &_hash_nodes[counter];
        thash_item->id = id;
        thash_item->i = i;
        thash_item->ppfInd = ppf_ind;

        hashtableInsertHashed(_hash_table, id, (void *)thash_item);

        counter++;
    }
    std::cout << std::endl;
}
int main(int argc, char** argv)
{
    FileStorage fsload(filename, FileStorage::READ);
    LoadHashTable(fsload.root());
    fsload.release();

    cout << "hash_table size: " << _hash_table->size << endl;

    int size = _hash_table->size;

    hashtable_int *_hash_T = hashtableCreate(size, NULL);
    THash *_hash_N = (THash *)calloc(size, sizeof(THash));

    *_hash_T = *_hash_table;
    *_hash_N = *_hash_nodes;

    cout << "hash_table B size: " << _hash_T->size << endl;

    free(_hash_nodes); _hash_nodes = 0;
    hashtableDestroy(_hash_table); _hash_table = 0;
    cout << "After, hash_table B size: " << _hash_T->size << endl;
    cout << "After, hash_table A size: " << _hash_table->size << endl;

    return 0;
}

