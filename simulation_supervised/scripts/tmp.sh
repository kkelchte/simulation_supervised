for iter_run in $(seq 1 3) ; do
	echo "./train_model_canyon.sh canyon_mob_$iter_run none 20 --batch_size 8"
	./train_model_canyon.sh canyon_mob_$iter_run none 20 --batch_size 8
	echo "./train_model_canyon.sh canyon_mob_auxd_$iter_run none 20 --auxiliary_depth True --batch_size 8"
	./train_model_canyon.sh canyon_mob_auxd_$iter_run none 20 --auxiliary_depth True --batch_size 8
	echo "./train_model_canyon.sh canyon_mob_nfc_$iter_run none 20 --n_fc True --batch_size 8"
	./train_model_canyon.sh canyon_mob_nfc_$iter_run none 20 --n_fc True --batch_size 8
	echo "./train_model_canyon.sh canyon_mob_nfc_auxd_$iter_run none 20 --auxiliary_depth True --n_fc True --show_depth True --batch_size 8"
	./train_model_canyon.sh canyon_mob_nfc_auxd_$iter_run none 20 --auxiliary_depth True --n_fc True --show_depth True --batch_size 8
	echo "./train_model_canyon.sh canyon_mob_nfc_auxo_$iter_run none 20 --auxiliary_odom True --n_fc True --show_odom True --batch_size 8"
	./train_model_canyon.sh canyon_mob_nfc_auxo_$iter_run none 20 --auxiliary_odom True --n_fc True --show_odom True --batch_size 8
	echo "./train_model_canyon.sh canyon_mob_nfc_auxod_$iter_run none 20 --auxiliary_odom True --n_fc True --show_odom True --auxiliary_depth True --show_depth True --batch_size 8"
	./train_model_canyon.sh canyon_mob_nfc_auxod_$iter_run none 20 --auxiliary_odom True --n_fc True --show_odom True --auxiliary_depth True --show_depth True --batch_size 8
done

echo "./evaluate_model_canyon.sh eva_canyon_mob canyon_mob_$iter_run 5 "
./evaluate_model_canyon.sh eva_canyon_mob canyon_mob_$iter_run 5 
echo "./evaluate_model_canyon.sh eva_canyon_mob_auxd canyon_mob_auxd_$iter_run 5 --auxiliary_depth True "
./evaluate_model_canyon.sh eva_canyon_mob_auxd canyon_mob_auxd_$iter_run 5 --auxiliary_depth True 
echo "./evaluate_model_canyon.sh eva_canyon_mob_nfc canyon_mob_nfc_$iter_run 5 --n_fc True "
./evaluate_model_canyon.sh eva_canyon_mob_nfc canyon_mob_nfc_$iter_run 5 --n_fc True 
echo "./evaluate_model_canyon.sh eva_canyon_mob_nfc_auxd canyon_mob_nfc_auxd_$iter_run 5 --auxiliary_depth True --n_fc True --show_depth True "
./evaluate_model_canyon.sh eva_canyon_mob_nfc_auxd canyon_mob_nfc_auxd_$iter_run 5 --auxiliary_depth True --n_fc True --show_depth True 
echo "./evaluate_model_canyon.sh eva_canyon_mob_nfc_auxo canyon_mob_nfc_auxo_$iter_run 5 --auxiliary_odom True --n_fc True --show_odom True "
./evaluate_model_canyon.sh eva_canyon_mob_nfc_auxo canyon_mob_nfc_auxo_$iter_run 5 --auxiliary_odom True --n_fc True --show_odom True 
echo "./evaluate_model_canyon.sh eva_canyon_mob_nfc_auxod canyon_mob_nfc_auxod_$iter_run 5 --auxiliary_odom True --n_fc True --show_odom True --auxiliary_depth True --show_depth True"
./evaluate_model_canyon.sh eva_canyon_mob_nfc_auxod canyon_mob_nfc_auxod_$iter_run 5 --auxiliary_odom True --n_fc True --show_odom True --auxiliary_depth True --show_depth True