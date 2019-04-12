for i in {1..10}
do
	cp -r /nfs/hal01/edalin/pybullet_minitaur_sim /nfs/hal01/edalin/minitaur_20secs/$i
	cd /nfs/hal01/edalin/minitaur_20secs/$i
	oarsub -l /nodes=1/core=32,walltime=170:00 "/nfs/hal01/edalin/miniconda3/conda-run minitaur /nfs/hal01/edalin/minitaur_20secs/$i/simulator_map_elites.py"
done
