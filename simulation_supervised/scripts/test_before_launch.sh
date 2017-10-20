start_time=$(date +%s)
rm -r /home/klaas/tensorflow/log/test_gt_fc
rm -r /home/klaas/tensorflow/log/test_mob_nfc
rm -r /home/klaas/tensorflow/log/test_auxd
rm -r /home/klaas/tensorflow/log/test_auxo
rm -r /home/klaas/tensorflow/log/test_auxod

./train_and_evaluate_model_debug.sh test_gt_fc None 1 --network fc_control --depth_input True --learning_rate 0.5
./train_and_evaluate_model_debug.sh test_mob_nfc None 1 --network mobile --model_path mobilenet --n_fc True --batch_size 8
./train_and_evaluate_model_debug.sh test_auxd None 1 --auxiliary_depth True --show_depth True
./train_and_evaluate_model_debug.sh test_auxo None 1 --auxiliary_odom True --n_fc True --show_odom True --batch_size 8
./train_and_evaluate_model_debug.sh test_auxod None 1 --auxiliary_odom True --auxiliary_depth True --n_fc True --show_odom True --batch_size 8
./train_and_evaluate_model_debug.sh test_auxod None 1 --auxiliary_odom True --n_fc True --show_odom True --batch_size 8 --auxiliary_depth True --show_depth True --concatenate_depth True --concatenate_odom True --odom_loss huber --depth_loss huber --plot_depth True
# ./train_and_evaluate_model_debug.sh test_lstm None 1 --network mobile_small --model_path mobilenet_small --lstm True --batch_size 6
end_time=$(date +%s)
echo "duration=$((end_time-start_time))"
for d in test_gt_fc test_mob_nfc test_auxd test_auxo test_auxod ; do
less /home/klaas/tensorflow/log/$d/xterm*
done
