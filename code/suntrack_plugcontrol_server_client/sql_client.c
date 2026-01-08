#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <mysql/mysql.h>
#include <stdbool.h>

#define BUF_SIZE 512
#define NAME_SIZE 512
#define ARR_CNT 64
#define DEVICE_NAME "KCCI"

void *send_msg(void *arg);
void *recv_msg(void *arg);
void error_handling(char *msg);

char name[NAME_SIZE] = "[Default]";
char msg[BUF_SIZE];

int main(int argc, char *argv[])
{
	int sock;
	struct sockaddr_in serv_addr;
	pthread_t snd_thread, rcv_thread;
	void *thread_return;

	if (argc != 4)
	{
		printf("Usage : %s <IP> <port> <name>\n", argv[0]);
		exit(1);
	}

	sprintf(name, "%s", argv[3]);

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock == -1)
		error_handling("socket() error");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(atoi(argv[2]));

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
		error_handling("connect() error");

	sprintf(msg, "[%s:PASSWD]", name);
	write(sock, msg, strlen(msg));
	pthread_create(&rcv_thread, NULL, recv_msg, (void *)&sock);
	pthread_create(&snd_thread, NULL, send_msg, (void *)&sock);

	pthread_join(snd_thread, &thread_return);
	pthread_join(rcv_thread, &thread_return);

	if (sock != -1)
		close(sock);
	return 0;
}

void *send_msg(void *arg)
{
	int *sock = (int *)arg;
	int str_len;
	int ret;
	fd_set initset, newset;
	struct timeval tv;
	char name_msg[NAME_SIZE + BUF_SIZE + 2];

	FD_ZERO(&initset);
	FD_SET(STDIN_FILENO, &initset);

	fputs("Input a message! [ID]msg (Default ID:ALLMSG)\n", stdout);
	while (1)
	{
		memset(msg, 0, sizeof(msg));
		name_msg[0] = '\0';
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		newset = initset;
		ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);
		if (FD_ISSET(STDIN_FILENO, &newset))
		{
			fgets(msg, BUF_SIZE, stdin);
			if (!strncmp(msg, "quit\n", 5))
			{
				*sock = -1;
				return NULL;
			}
			else if (msg[0] != '[')
			{
				strcat(name_msg, "[ALLMSG]");
				strcat(name_msg, msg);
			}
			else
				strcpy(name_msg, msg);
			if (write(*sock, name_msg, strlen(name_msg)) <= 0)
			{
				*sock = -1;
				return NULL;
			}
		}
		if (ret == 0)
		{
			if (*sock == -1)
				return NULL;
		}
	}
}

void *recv_msg(void *arg)
{
	MYSQL *conn;
	MYSQL_ROW sqlrow;
	int res;
	char sql_cmd[512] = {0};
	char *host = "10.10.14.77";
	char *user = "iot";
	char *pass = "pwiot";
	char *dbname = "iotdb";

	int *sock = (int *)arg;
	int i;
	char *pToken;
	char *pArray[ARR_CNT] = {0};

	char name_msg[NAME_SIZE + BUF_SIZE + 1];
	int str_len;

	conn = mysql_init(NULL);

	puts("MYSQL startup");
	if (!(mysql_real_connect(conn, host, user, pass, dbname, 0, NULL, 0)))
	{
		fprintf(stderr, "ERROR : %s[%d]\n", mysql_error(conn), mysql_errno(conn));
		exit(1);
	}
	else
		printf("Connection Successful!\n\n");

	while (1)
	{
		memset(name_msg, 0x0, sizeof(name_msg));
		str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE);
		if (str_len <= 0)
		{
			*sock = -1;
			return NULL;
		}
		fputs(name_msg, stdout);
		//		name_msg[str_len-1] = 0;   //'\n' 제거
		name_msg[strcspn(name_msg, "\n")] = '\0';

		pToken = strtok(name_msg, "[:@]");
		i = 0;
		while (pToken != NULL)
		{
			pArray[i] = pToken;
			if (++i >= ARR_CNT)
				break;
			pToken = strtok(NULL, "[:@]");
		}
		if (!strcmp(pArray[1], "SENSOR"))
		{
			// 값/플래그
			char pos[8] = {0};
			double TL = 0, TM = 0, TR = 0, ML = 0, MR = 0, BL = 0, BM = 0, BR = 0, Solar = 0;
			bool gotPos = false, gotTL = false, gotTM = false, gotTR = false, gotML = false, gotMR = false, gotBL = false, gotBM = false, gotBR = false, gotSolar = false;

			// pArray에서 "SENSOR" 위치 찾기
			int cmd_idx = -1;
			for (int k = 0; k < i; k++)
			{
				if (pArray[k] && strcmp(pArray[k], "SENSOR") == 0)
				{
					cmd_idx = k;
					break;
				}
			}
			if (cmd_idx == -1)
				goto after_sensor;

			// SENSOR 다음부터 (key,val) 쌍으로 순회
			for (int j = cmd_idx + 1; j + 1 < i; j += 2)
			{
				char *key = pArray[j];
				char *val = pArray[j + 1];
				size_t L = strlen(val);
				if (L && val[L - 1] == '\r')
					val[L - 1] = '\0';

				if (!strcasecmp(key, "Pos"))
				{
					strncpy(pos, val, sizeof(pos) - 1);
					gotPos = true;
				}
				else if (!strcasecmp(key, "TL"))
				{
					TL = atof(val);
					gotTL = true;
				}
				else if (!strcasecmp(key, "TM"))
				{
					TM = atof(val);
					gotTM = true;
				}
				else if (!strcasecmp(key, "TR"))
				{
					TR = atof(val);
					gotTR = true;
				}
				else if (!strcasecmp(key, "ML"))
				{
					ML = atof(val);
					gotML = true;
				}
				else if (!strcasecmp(key, "MR"))
				{
					MR = atof(val);
					gotMR = true;
				}
				else if (!strcasecmp(key, "BL"))
				{
					BL = atof(val);
					gotBL = true;
				}
				else if (!strcasecmp(key, "BM"))
				{
					BM = atof(val);
					gotBM = true;
				}
				else if (!strcasecmp(key, "BR"))
				{
					BR = atof(val);
					gotBR = true;
				}
				else if (!strcasecmp(key, "Solar"))
				{
					Solar = atof(val);
					gotSolar = true;
				}
			}

			if (!(gotPos && gotSolar && gotTL && gotTM && gotTR && gotML && gotMR && gotBL && gotBM && gotBR))
			{
				fprintf(stderr, "[SENSOR] missing fields; skip insert.\n");
				goto after_sensor;
			}

			char pos_esc[16] = {0};
			mysql_real_escape_string(conn, pos_esc, pos, strlen(pos));

			char sql_cmd[512];
			snprintf(sql_cmd, sizeof(sql_cmd),
					 "INSERT INTO sensor "
					 "(ts, name, pos, solar, cds0, cds1, cds2, cds3, cds4, cds5, cds6, cds7) "
					 "VALUES (NOW(), '%s', '%s', %.3f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f)",
					 DEVICE_NAME, pos_esc, Solar, TL, TM, TR, ML, MR, BL, BM, BR);

			int res = mysql_query(conn, sql_cmd);
			if (!res)
				printf("[SENSOR] inserted %lu rows\n", (unsigned long)mysql_affected_rows(conn));
			else
				fprintf(stderr, "ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));

		after_sensor:;
		}
		//[KSH_SQL]GETDB@LAMP
		//[KSH_SQL]SETDB@LAMP@ON
		//[KSH_SQL]SETDB@LAMP@ON@KSH_LIN
		if (!strcmp(pArray[1], "GETDB") && i == 3)
		{
			char name_esc[512];
			mysql_real_escape_string(conn, name_esc, pArray[2], (unsigned long)strlen(pArray[2]));

			snprintf(sql_cmd, sizeof(sql_cmd),
					 "SELECT "
					 "cds0, cds1, cds2, cds3, cds4, cds5, cds6, cds7, "
					 "pos, solar, DATE_FORMAT(ts,'%%Y-%%m-%%d_%%H:%%i:%%s') AS ts_fmt "
					 "FROM sensor WHERE name='%s' "
					 "ORDER BY ts DESC LIMIT 1",
					 name_esc);

			if (mysql_query(conn, sql_cmd))
			{
				fprintf(stderr, "GETDB query err: %s\n", mysql_error(conn));
				snprintf(sql_cmd, sizeof(sql_cmd), "[%s]GETDB@%s@ERR@QUERY\n", pArray[0], pArray[2]);
				write(*sock, sql_cmd, strlen(sql_cmd));
				break;
			}

			MYSQL_RES *result = mysql_store_result(conn);
			if (!result)
			{
				fprintf(stderr, "GETDB store_result err: %s\r\n", mysql_error(conn));
				snprintf(sql_cmd, sizeof(sql_cmd), "[%s]GETDB@%s@ERR@RESULT\n", pArray[0], pArray[2]);
				write(*sock, sql_cmd, strlen(sql_cmd));
				break;
			}

			MYSQL_ROW row = mysql_fetch_row(result);
			if (!row)
			{
				snprintf(sql_cmd, sizeof(sql_cmd), "[%s]GETDB@%s@ERR@NoData\n", pArray[0], pArray[2]);
				write(*sock, sql_cmd, strlen(sql_cmd));
				mysql_free_result(result);
				break;
			}

			// 응답: [LT_SQL]GETDB@name@TL@..@...@BR@..@Pos@..@Solar@..@TS@yyyy-mm-dd_HH:MM:SS
			snprintf(sql_cmd, sizeof(sql_cmd),
					 "[%s]GETDB@%s"
					 "@Pos@%s@Solar@%s\n",
					 pArray[0], pArray[2],
					 row[8], row[9]);

			write(*sock, sql_cmd, strlen(sql_cmd));
			mysql_free_result(result);
		}
else if (!strcmp(pArray[1], "SETDB"))
		 {
		 	sprintf(sql_cmd, "update device set value='%s' where name='%s'", pArray[3], pArray[2]);

		 	res = mysql_query(conn, sql_cmd);
		 	if (!res)
		 	{
		 		if (i == 4)
		 			sprintf(sql_cmd, "[%s]%s@%s@%s\n", pArray[0], pArray[1], pArray[2], pArray[3]);
		 		else if (i == 5)
		 			sprintf(sql_cmd, "[%s]%s@%s\n", pArray[4], pArray[2], pArray[3]);
				else
		 			continue;

		 		printf("inserted %lu rows\n", (unsigned long)mysql_affected_rows(conn));
		 		write(*sock, sql_cmd, strlen(sql_cmd));
		 	}
		 	else
		 		fprintf(stderr, "ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));
		 }


		else if ((!strcmp(pArray[1], "PLUG1") || !strcmp(pArray[1], "PLUG2")) && i == 3)
		 {
    		// 1. MySQL에 상태 업데이트
    		sprintf(sql_cmd, "update device set value='%s', date=now(), time=now() where name='%s'", pArray[2],  pArray[1]);
   		 res = mysql_query(conn, sql_cmd);
    		if (!res)
			{
    		    printf("Updated %s to %s in MySQL\n",  pArray[1], pArray[2]);
   			}
			else
			{
    		    fprintf(stderr, "SQL ERROR: %s [%d]\n", mysql_error(conn), mysql_errno(conn));
			}

			 // 2. 아두이노에 그대로 메시지 전달
			 sprintf(sql_cmd, "[%s]%s@%s\n", pArray[0], pArray[1], pArray[2]);
			 write(*sock, sql_cmd, strlen(sql_cmd));
		}
	}
	mysql_close(conn);
}

void error_handling(char *msg)
{
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(1);
}
