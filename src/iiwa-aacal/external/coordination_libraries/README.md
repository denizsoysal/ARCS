# Coordination Libraries
## Description
This project implements common coordination datastructures and mechanisms (petrinet, fsm, lcsm, DAG).
```top-level_4_state_lcsm``` is a specific example of how an fsm is made with the fsm library.

## Dependencies

* CGraph
    * Generic in-memory graph library: For DAG, Flow chart and petrinet construction and manipulation
    * Linux install ```$ sudo apt install libgraphviz-dev```

* ut_hash
    * C hash table implementation (used for making dictionnaries)
    * place ut_hash.h in your system path
    * Download found [here](https://troydhanson.github.io/uthash/)
    
## build

The libraries will be installed in your home directory in folder ```coordination-libs```

After installation of the dependencies, do
    
```bash
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=</path/to/local_install_root> ..
$ make 
& make install
```

