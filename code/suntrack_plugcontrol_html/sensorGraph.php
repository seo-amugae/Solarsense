<!DOCTYPE html>
<html>
<head>
	<meta charset = "UTF-8">
	<meta http-equiv = "refresh" content = "5">
</head>
<?php
	$conn = mysqli_connect("localhost", "iot", "pwiot");
	mysqli_select_db($conn, "iotdb");
    $query = "
    SELECT ts, solar, cds0, cds1, cds2, cds3, cds4, cds5, cds6, cds7
    FROM (
        SELECT ts, solar, cds0, cds1, cds2, cds3, cds4, cds5, cds6, cds7
        FROM sensor
        ORDER BY ts DESC
        LIMIT 200
    ) AS recent
    ORDER BY ts ASC
    ";
    $result = mysqli_query($conn, $query);

    // 첫 행은 헤더: 시간이 첫 컬럼
    $data = array(array('time', 'solar', 'cds0', 'cds1', 'cds2', 'cds3', 'cds4', 'cds5', 'cds6', 'cds7'));

    if ($result) {
        while ($row = mysqli_fetch_assoc($result)) {
            $data[] = array(
            $row['ts'],                   // JS에서 Date 객체로 변환 예정
            (float)$row['solar'],
            (float)$row['cds0'],
            (float)$row['cds1'],
            (float)$row['cds2'],
            (float)$row['cds3'],
            (float)$row['cds4'],
            (float)$row['cds5'],
            (float)$row['cds6'],
            (float)$row['cds7']
            );
        }
    }

    $options = [
    'title'  => 'sensor_bt (solar & cds0~7)',
    'height' => 800,
    'legend' => ['position'=>'bottom'],
    'curveType' => 'function',        // ← 곡선 처리
    'lineWidth' => 2,                 // 선 두께
    'pointSize' => 3,                 // 포인트 크기(0이면 숨김)
    'dataOpacity' => 0.9,
    'backgroundColor' => 'transparent',
    'chartArea' => ['left'=>60,'right'=>20,'top'=>40,'bottom'=>60],
    'hAxis' => [
        'slantedText' => true,
        'gridlines' => ['color' => '#eee']
    ],
    'vAxis' => [
        'gridlines' => ['color' => '#f3f3f3'],
        'minorGridlines' => ['color' => '#fafafa']
    ],
    'focusTarget' => 'category' // 동일 x축 값 하이라이트
    ];
?>

<script src="//www.google.com/jsapi"></script>
<script>
var data = <?=json_encode($data) ?>;
var options = <?= json_encode($options) ?>;

google.load('visualization', '1.0', {'packages':['corechart']});

google.setOnLoadCallback(function() {
	var chart = new google.visualization.LineChart(document.querySelector('#chart_div'));
	chart.draw(google.visualization.arrayToDataTable(data), options);
	});
	</script>
<div id="chart_div"></div>
