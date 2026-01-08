<!DOCTYPE html>
<html>
<head>
	<meta charset = "UTF-8">
	<meta http-equiv = "refresh" content = "5">
	<style type = "text/css">
		.spec{
			text-align:center;
		}
		.con{
			text-align:left;
		}
		</style>
</head>
<?php
    $conn = mysqli_connect("localhost", "iot", "pwiot");
    mysqli_select_db($conn, "iotdb");

    $result = mysqli_query(
        $conn,
        "SELECT id,name,ts,pos,solar,cds0,cds1,cds2,cds3,cds4,cds5,cds6,cds7
         FROM sensor
         ORDER BY id DESC
         LIMIT 200"
    );
?>
<body>
	<table border = '1' style = "width = 30%" align = "center">
	<tr align = "center">
    <th>id</th><th>name</th><th>ts</th><th>pos</th><th>solar</th>
    <th>cds0</th><th>cds1</th><th>cds2</th><th>cds3</th>
    <th>cds4</th><th>cds5</th><th>cds6</th><th>cds7</th>
	</tr>

	<?php
		while($row = mysqli_fetch_assoc($result)){
			echo "<tr class='con'>";
			echo "<td>{$row['id']}</td>";
			echo "<td>{$row['name']}</td>";
			echo "<td>{$row['ts']}</td>";
			echo "<td>{$row['pos']}</td>";
			echo "<td>{$row['solar']}V</td>";
			echo "<td>{$row['cds0']}%</td>";
			echo "<td>{$row['cds1']}%</td>";
			echo "<td>{$row['cds2']}%</td>";
			echo "<td>{$row['cds3']}%</td>";
			echo "<td>{$row['cds4']}%</td>";
			echo "<td>{$row['cds5']}%</td>";
			echo "<td>{$row['cds6']}%</td>";
			echo "<td>{$row['cds7']}%</td>";
			echo "</tr>";
		}
		mysqli_close($conn);
	?>
	</table>
</body>
</html>
