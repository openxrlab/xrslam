# Benchmark

We run our algorithm on EuRoC dataset on Ubuntu18.04 and macOS 10.14. And make comparisons with  VINS-Mono, which is one of the state-of-the-art VIO systems. We analyze the accuracy of the algorithm by comparing the root mean squared error (RMSE) of the absolute trajectory error (ATE). ATE is given by the simple difference between the estimated trajectory and ground truth after it has been aligned so that it has a minimal error, and RMSE is the standard deviation of the residuals (prediction errors). The lower the RMSE, the better a given trajectory is able to fit its ground truth. We use "evo" tool to evaluate and compare the trajectory output of odometry and SLAM algorithms, and for more information please refer to [euroc evaluation](./tutorials/euroc_evaluation.md). As shown in the following table, XRSLAM has higher accuracy than VINS-Mono (without loop). The average results for visual-inertial algorithms are bolded, and the following figures show the trajectory of XRSLAM on Vicon Room 1 01 sequence.

<table >
	<tr>
	    <th rowspan="2">Sequence</th>
        <th colspan="2">APE(m)</th>
        <th colspan="2">ARE(deg)</th>
	</tr >
	<tr>
	    <th>XRSLAM</th>
	    <th>VINS-Mono</th>
	    <th>XRSLAM</th>
	    <th>VINS-Mono</th>
	</tr >
	<tr >
	    <td align="center">MH_01</td>
        <td  align="center">0.147</td> <td align="center">0.154</td> <td align="center">2.516</td> <td align="center">1.516</td>
	</tr>
	<tr >
	    <td align="center">MH_02</td>
        <td align="center">0.077</td> <td align="center">0.178</td> <td align="center">1.707</td> <td align="center">2.309</td>
	</tr>  
	<tr >
	    <td align="center">MH_03</td>
        <td align="center">0.154</td> <td align="center">0.195</td> <td align="center">1.554</td> <td align="center">1.646</td>
	</tr>  
	<tr >
	    <td align="center">MH_04</td>
        <td align="center">0.269</td> <td align="center">0.376</td> <td align="center">1.344</td> <td align="center">1.431</td>
	</tr>
	<tr >
	    <td align="center">MH_05</td>
        <td align="center">0.252</td> <td align="center">0.300</td> <td align="center">0.740</td> <td align="center">0.782</td>
	</tr>
	<tr >
	    <td align="center">V1_01</td>
        <td align="center">0.063</td> <td align="center">0.088</td> <td align="center">5.646</td> <td align="center">6.338</td>
	</tr>
	<tr >
	    <td align="center">V1_02</td>
        <td align="center">0.097</td> <td align="center">0.111</td> <td align="center">1.877</td> <td align="center">3.278</td>
	</tr>
	<tr >
	    <td align="center">V1_03</td>
        <td align="center">0.102</td> <td align="center">0.187</td> <td align="center">2.190</td> <td align="center">6.211</td>
	</tr>
	<tr >
	    <td align="center">V2_01</td>
        <td align="center">0.066</td> <td align="center">0.082</td> <td align="center">1.301</td> <td align="center">2.137</td>
	</tr>
	<tr >
	    <td align="center">V2_02</td>
        <td align="center">0.092</td> <td align="center">0.149</td> <td align="center">1.521</td> <td align="center">3.976</td>
	</tr>
	<tr >
	    <td align="center">V2_03</td>
        <td align="center">0.193</td> <td align="center">0.287</td> <td align="center">1.592</td> <td align="center">3.331</td>
	</tr>
	<tr >
	    <td align="center">Average</td>
        <th>0.137</th> <th>0.192</th> <th>1.998</th> <th>2.995</th>
	</tr>



---


<div align='center'><img src="../images/PC-Player.png" width="60%" height="100%"><img src="../images/trajectory.png" width="39.8%" height="20%"></div>

---
