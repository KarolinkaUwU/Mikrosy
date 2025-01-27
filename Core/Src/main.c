/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "string.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <QMC5883.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Enum - szkielet maszyny stanów
enum frame_states {
	WAIT_FOR_MESSAGE,
	FIND_FRAME_START,
	COLLECT_FRAME,
	CHECK_RECEIVER_SENDER,
	CHECK_COMMAND_LENGTH,
	CHECK_CHECKSUM,
	ANALYZE_COMMAND
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WAIT_FOR_MESSAGE 1
#define FIND_FRAME_START 2
#define COLLECT_FRAME 3
#define CHECK_RECEIVER_SENDER 4
#define CHECK_COMMAND_LENGTH 5
#define CHECK_CHECKSUM 6
#define ANALYZE_COMMAND 7

// Długość bufora odbiorczego/nadawczego
#define buff_length 1000

// Dzielnik, przez który dzielona z resztą (modulo %) jest suma wartości dziesiętnej wszystkich znaków w polu 'dane' ramki
#define checksum_div 32\

// Minimalna i maksymalna długość ramki
#define frame_min_length 12
#define frame_max_length 268

#define WAIT_FOR_MESSAGE 1
#define FIND_FRAME_START 2
#define COLLECT_FRAME 3
#define CHECK_COMMAND_LENGTH 5
#define CHECK_CHECKSUM 6
#define ANALYZE_COMMAND 7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

QMC_t sensor;
float Compas_Value;

// Urządzenia stm - to stm, usr to użytkownik
char device_address[3] = "STM";
char source_address[3] = "USR";

// Ilość znaków pobranych z otrzymanej wiadomości. Znaki pobierane są gdy znaleziony zostanie początek ramki aż do napotkania znaku końca ramki. Oba te znaki nie są pobierane
__IO uint16_t frame_length;

// Tablica, do której pobierana jest ramka danych
__IO uint16_t frame[buff_length];

// Ilość znaków w polu 'dane' ramki, które będą komendą do wykonania dla programu
__IO uint16_t command_length;

// Ilość znaków składających się na otrzymaną wiadomość (pomijając znak końca transmisji)
__IO uint16_t message_length;

// Tablica, do której pobierana jest z bufora odbiorczego otrzymana wiadomość
__IO uint8_t message[buff_length];

// Wskaźnik pomocniczy dla odebranej wiadomości,
	// Po pierwsze pozwala na sprawdzenie każdego pojedynczego odebranego znaku, czy nie jest on znakiem końca transmisji
	// Po drugie, pozwala na ponowne analizowanie wiadomości pod kątem szukania ramki, gdy jedna ramka już została przeanalizowana lub napodkano kilka znaków początku ramki
__IO uint8_t message_idx;

// Bufor odbiorczy
__IO uint8_t buf_rx[buff_length];
// Wskaźnik pierwszego wolnego miejsca
__IO uint16_t rx_empty = 0;
// Wskaźnik miejsca ktore nie jest jeszcze zanalizowane
__IO uint16_t rx_busy = 0 ;


// To samo ale bufor nadawczy
__IO uint8_t buf_tx[buff_length];
__IO uint16_t tx_empty = 0;
__IO uint16_t tx_busy = 0;

// zmienna aktualnego stanu analizy
__IO enum frame_states frame_state = WAIT_FOR_MESSAGE;

int16_t x, y, z;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// Callbacki
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

// Ogólne funkcje użytkowe
void send(char *msgToSend, ...);
void increase_rx_empty();
void increase_rx_busy();
void increase_tx_empty();
void increase_tx_busy();
void reset_frame_state();
void send_answer_message(char* error_msg, ...);
void change_int_to_char_arr(char *array_reference, uint8_t err_checksum);

uint8_t get_char();
uint8_t rx_has_data();
uint8_t tx_has_data();
uint8_t is_char_frame_start(uint8_t *single_char);
uint8_t is_char_frame_end(uint8_t *single_char);
uint8_t is_string_alphanumeric(char tmp[], int array_size);
uint8_t get_message(char *array_reference);
uint8_t is_char_endmessage(char single_char);

// Funkcje typowo pod czujnik

uint8_t QMC_init(QMC_t *qmc,I2C_HandleTypeDef *i2c,uint8_t Output_Data_Rate)
{
	uint8_t array[2];
	qmc->i2c=i2c;
	qmc->Control_Register=0x11;
	array[0]=1;
	array[1]=qmc->Control_Register;

	if(Output_Data_Rate==200)qmc->Control_Register|=0b00001100;
	else if(Output_Data_Rate==100)qmc->Control_Register|=0b00001000;
	else if(Output_Data_Rate==50)qmc->Control_Register|=0b00000100;
	else if(Output_Data_Rate==10)qmc->Control_Register|=0b00000000;
	else qmc->Control_Register|=0b00001100;

	if(HAL_I2C_Mem_Write(qmc->i2c, 0x1A, 0x0B, 1, &array[0], 1, 100)!=HAL_OK)return 1;
	if(HAL_I2C_Mem_Write(qmc->i2c, 0x1A, 0x09, 1, &array[1], 1, 100)!=HAL_OK)return 1;

	return 0;
}

uint8_t QMC_read(QMC_t *qmc, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    if (HAL_I2C_Mem_Read_DMA(qmc->i2c, 0x1A, 0x00, I2C_MEMADD_SIZE_8BIT, data, 6) != HAL_OK)
        return 1;

    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);

    qmc->heading = atan2f((float)*y, (float)*x) * 57.3f; // Konwersja na stopnie
    return 0;
}


float QMC_readHeading(QMC_t *qmc)
{
    int16_t x, y, z;
    QMC_read(qmc, &x, &y, &z);
    return qmc->heading;
}


uint8_t QMC_Standby(QMC_t *qmc)
{
    uint8_t array[1] = {0};
    if (HAL_I2C_Mem_Write_DMA(qmc->i2c, 0x1A, 0x09, 1, &array[0], 1) != HAL_OK)
        return 1;
    return 0;
}


uint8_t QMC_Reset(QMC_t *qmc)
{
    uint8_t array[1] = {0x80};
    return HAL_I2C_Mem_Write_DMA(qmc->i2c, 0x1A, 0x0A, I2C_MEMADD_SIZE_8BIT, &array[0], 1) != HAL_OK;
}




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Sprawdzenie czy bufor odbiorczy zawiera jakieś dane
uint8_t rx_has_data() {
	// Jeśli empty i busy wskazuja to samo miejsce to znaczy, że pusty
	if (rx_empty == rx_busy)
		return 0;
	else
		return 1;
}

// To samo do bufora nadawczego
uint8_t tx_has_data() {
	if (tx_empty == tx_busy)
		return 0;
	else
		return 1;
}

void increase_rx_empty(){
	rx_empty++;
	rx_empty %= buff_length;
}

void increase_rx_busy(){
	rx_busy++;
	rx_busy %= buff_length;
}

void increase_tx_empty(){
	tx_empty++;
	tx_empty %= buff_length;
}

void increase_tx_busy(){
	tx_busy++;
	tx_busy %= buff_length;
}

uint8_t get_char() {
	uint8_t tmp;
	if (rx_has_data() == 1) {
		tmp = buf_rx[rx_busy];
		increase_rx_busy();
		return tmp;
	}
	else {
		return 0;
	}
}

uint8_t is_char_endmessage(char single_char){
	if (single_char == 10 || single_char == 13)
		return 1;
	else
		return 0;
}


uint8_t get_message(char *array_reference) {
	static uint8_t tmp_buff[buff_length];
	static uint8_t idx = 0;
	uint8_t mess_length;

	while (rx_has_data() == 1) {
		tmp_buff[idx] = getchar();

		if(is_char_endmessage(tmp_buff[idx])) {
			tmp_buff[idx] = 0;

			for (int i = 0; i <= idx; i++)
				array_reference[i] = tmp_buff[i];

			mess_length = idx;
			idx = 0;
			return mess_length;
		}
		else {
			idx++;
			if (idx >= buff_length)
				return 0;
		}
	}
	return 0;
}

void change_int_to_char_arr(char *array_reference, uint8_t int_val) {
	if (int_val > 100)
		sprintf(array_reference, "%d", int_val);
	else if (int_val > 10)
		sprintf(array_reference,"0%d", int_val);
	else
		sprintf(array_reference,"00%d", int_val);
}

// Z STM do PC wysyłanie danych

void send(char *msgToSend, ...) {
	// tablica pomocniczna
	char data_to_send[buff_length];
	// wskaźnik pomocniczy
	uint16_t idx;
	// lista argumentów
	va_list arglist;

	// kopiowanie znaków (char) do tablicy
	va_start(arglist, msgToSend);
	vsprintf(data_to_send, msgToSend, arglist);
	va_end(arglist);

	// Ustawienie wskaźnika pomocnicznego na pierwsze wolne miejsce
	idx = tx_empty;

	// wrzucanie danych do bufora nadawczego
	for (int i = 0; i < strlen(data_to_send); i++) {
		buf_tx[idx] = data_to_send[i];
		idx++;
		idx %= buff_length;
	}

	__disable_irq();

	if (tx_has_data() == 0 && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
		tx_empty = idx;
		HAL_UART_Transmit_IT(&huart2, &buf_tx[tx_busy], 1);
		increase_tx_busy();
	}
	else
		tx_empty = idx;
	__enable_irq();
}

// Z STM do PC
void send_answer_message(char* ans_msg, ...) {
	char answer_data[buff_length];
	char answer_msg[buff_length];
	uint16_t answer_length;
	va_list arglist;

	va_start(arglist, ans_msg);
	vsprintf(answer_data, ans_msg, arglist);
	va_end(arglist);

	answer_length = strlen(answer_data);

	uint8_t ans_checksum = 0;
	uint16_t ans_checksum_sum = 0;

	for (int i = 0; i < answer_length; i++)
		ans_checksum_sum += answer_data[i];

	ans_checksum = ans_checksum_sum % checksum_div;

	char ans_command_len_arr[3];
	change_int_to_char_arr(ans_command_len_arr, answer_length);
	char ans_checksum_arr[3];
	change_int_to_char_arr(ans_checksum_arr, ans_checksum);
	sprintf(answer_msg, "\%s%s%s%s%s\*" , device_address, source_address, ans_command_len_arr, ans_msg, ans_checksum_arr);
	send(answer_msg);
}



//odpowiedni znak!
uint8_t is_char_frame_start(uint8_t *single_char) {
	if (single_char == 0x3A)
		return 1;
	else
		return 0;
}


//odpowiedni znak!
uint8_t is_char_frame_end(uint8_t *single_char) {
	if (single_char == 0x3B)
		return 1;
	else
		return 0;
}

void reset_frame_state() {
	frame_length = 0;
	frame_state = FIND_FRAME_START;
}

uint8_t is_string_alphanumeric(char tmp[], int array_size) {
	for (int i = 0; i < array_size; i++) {
		if (!(tmp[i] >= '0' && tmp[i] <= '9'))
			return 0;
	}
	return 1;
}

// Callbacki

//Nadawczy
void HAL_UART_RxCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		increase_rx_empty();
		HAL_UART_Receive_IT(&huart2, &buf_rx[rx_empty], 1);
	}
}

//Odbiorczy
void HAL_UART_TxCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		if (tx_has_data() == 1){
			static uint8_t tmp;
			tmp = buf_tx[tx_busy];
			increase_tx_busy();
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &buf_rx[rx_empty], 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Program do switcha wchodzi tylko wtedy, gdy:
	  // 1. Gdy są dane do przeanalizowania - rx_has_data() == 1
	  // 2. Analiza danych została rozpoczęta (frame_state != WAIT_FOR_MESSAGE && rx_has_data() == 0)
	  // W momencie, gdy analiza trwa, a przybędą nowe dane, program będzie wykonywał warunek numer 1, lecz frame_state będzie dalej kontynuowało analizę uprzednich danych
	  // Aż wszystkie dane z tablic 'frame' i 'message' nie zostaną przeanalizowane
	  			if (rx_has_data() || (frame_state != WAIT_FOR_MESSAGE && rx_has_data() == 0)) {
	  				// Pojedynczy znak, do którego zczytywane będą dane
	  				uint8_t single_char;

	  				// Tablica pomocnicza
	  				char temp[3];

	  				switch (frame_state) {

	  				// Stan ten odpowiada za czekanie na wiadomość oraz pobieranie otrzymanych znaków aż do otrzymania znaku końca transmisji
	  				// Gdy taki znak nadejdzie, cała wiadomość kolekcjonowana jest do tablicy 'message' i stan zmieniany jest na FIND_FRAME_START
	  				case WAIT_FOR_MESSAGE:

	  					// Sprawdzanie, czy są nowe, nieprzeanalizowane jeszcze dane
	  					if (message_idx == rx_empty)
	  						break;

	  					// Pobieranie pojedynczego znaku
	  					single_char = buf_rx[message_idx];

	  					// Inkrementowanie pojedynczego wskaźnika
	  					message_idx++;
	  					message_idx %= buff_length;

	  					// Sprawdzanie, czy znak nie jest znakiem końca transmisji
	  					if (is_char_endmessage(single_char)) {
	  						message_length = get_message(message);

	  						// Sprawdzanie, czy nie doszło do przepełnienia tablicy pomocniczej zbierającej otrzymaną wiadomość
	  						if (message_length > 0) {
	  							message_idx = 0;
	  							frame_state = FIND_FRAME_START;
	  						}
	  						}
	  					break;

	  					// Stan odpowiada za poszukiwanie znaku początku ramki
	  					// Jeżeli początek zostanie odnaleziony, analiza przechodzi do następnego stanu
	  				case FIND_FRAME_START:

	  					// Iterowanie zaczyna się zawsze od 'message_idx', czyli miejsca z ostatnią przeanalizowana daną z tablicy 'message'
	  					for (int i = message_idx; i < message_length; i++) {
	  						// Jeżeli napotkano znak początku ramki
	  						if (is_char_frame_start(message[i])) {
	  							// Preinkrementacja i (żeby pominąć znak początku ramki w kolekcjonowaniu), jeżeli będzie >= message_length, to znaczy, że było to ostatnie okrążenie i ostatni znak w wiadomości
	  							// Jeżeli nie, to przypisujemy to i do message_idx i jedziemy dalej
	  							if (++i < message_length) {

	  								// Wartość i przypisywana jest do 'message_idx', żeby zacząć kolekcjonować ramkę od następnego miejsca po znaku rozpoczęcia ramki
	  								// Pomijanie znaku i przejście do następnego stanu
	  								message_idx = i;
	  								frame_state = COLLECT_FRAME;
	  								break;
	  							}
	  						}
	  					}

	  					// Gdy nie znaleziono znaku początku ramki w wiadomości po iteracji pętlą for po tablicy 'message'
	  					if (frame_state == FIND_FRAME_START) {
	  						frame_state = WAIT_FOR_MESSAGE;
	  						message_idx = rx_busy;
	  					}
	  					break;

	  					// Stan odpowiada za skolekcjonowanie całej ramki z otrzymanej wiadomości
	  					// Iterację zaczyna się od miejsca + 1, w którym wystąpił znak początku ramki
	  					// Każdy znak należy sprawdzić pod kątem, czy nie jest on kolejnym znakiem początku ramki
	  					// Jeżeli tak, należy zapisać miejsce jego występowania do wskaźnika 'message_idx', a następnie wrócić do stanu FIND_FRAME_START
	  					// Każdy znak należy także sprawdzić pod kątem, czy nie jest on znakiem końca ramki
	  					// Jeżeli tak, należy sprawdzić, czy skolekcjonowana ramka jest dłuższa, niż minimum znaków, z których ramka musi zostać zbudowana
	  					// bo nie ma sensu analizować dalej ramki, która i tak będzie błędna
	  					// Jeżeli długość skolekcjonowanej ramki > minimum, do wskaźnika message_idx przypisywane jest ++i, które oznacza, na którym znaku
	  					// zakończyła się analiza danych z tablicy 'message', przy czym pomija się znak końca ramki, dlatego ++i (żeby zacząć analizę od następnego znaku)
	  					// Jeżeli znak nie jest znakiem ani początku, ani końca ramki, jest kolekcjonowany do tablicy 'frame', a ilość skomplementowanych danych w zmiennej 'frame_length'
	  					// jest inkrementowana
	  						// Należy jednak sprawdzać, czy skolekcjonowana już teraz ramka nie przekroczyła maksymalnej długości, jeżeli tak, należy wrócić do stanu FIND_FRAME_START,
	  					// bo nie ma sensu analizować ramki, która wiadomo, że będzie nieprawidłowa, bo zawiera więcej znaków, niż powinna
	  				case COLLECT_FRAME:

	  					// Iterowanie po otrzymanej wiadomości, żeby znaleźć ramkę
	  					for (int i = message_idx; i < message_length; i++) {
	  						// Pobranie pojedynczego znaku
	  							single_char = message[i];

	  							// Jeżeli znak jest kolejnym znakiem początku
	  							if (is_char_frame_start(single_char)) {
	  								// Powrót do początku analizy
	  								message_idx = i;
	  								frame_state = FIND_FRAME_START;
	  								frame_length = 0;
	  						break;
	  							}
	  							// Jeżeli znak jest znakiem końca ramki
	  							else if (is_char_frame_end(single_char)) {
	  								// Jeżeli skolekcjonowana ramka jest mniejsza niż jej minimalna długość
	  								if (frame_length < frame_min_length) {
	  									// Powrót do początku analizy
	  									message_idx = ++i;
	  									reset_frame_state();
	  									break;
	  								}

	  								// Pominięcie znaku końca ramki i przejście do kolejnego stanu
	  								message_idx = ++i;
	  								frame_state = CHECK_RECEIVER_SENDER;
	  								break;
	  							}
	  							// Jeżeli znak jest innym znakiem niż początek/koniec ramki
	  							else {
	  								// Kolekcjonowanie ramki
	  									frame[frame_length] = single_char;
	  									frame_length++;

	  										// Jeżeli skolekcjonowana ramka przekroczyła maksymalną długość (min_frame_length + 256 (dane w protokole))
	  									if (frame_length >= frame_max_length) {
	  										// Powrót do początku analizy
	  										message_idx = ++i;
	  										break;
	  									}
	  							}
	  					}

	  					// Jeżeli iterowano po buforze i nie znaleziono znaku końca ramki (a znaleziono początek w poprzednim stanie) następuje powrót do czekania na dane
	  					// Lub przerwano iterowanie, lecz nie zmieniono stanu na następny
	  					if (frame_state == COLLECT_FRAME)
	  						reset_frame_state();
	  					break;

	  					// Stan ten odpowiada za sprawdzenie nadawcy/odbiorcy w ramce
	  					// Jeżeli ramka jest skolekcjonowana, należy najpierw sprawdzić, czy 'odbiorca' z ramki jest taki sam jak 'device_address', czyli urządzenie, na którym uruchomiony jest program
	  					// Jeżeli tak, należy skopiować 3 znaki odpowiadające 'nadawcy' ramki i przejść do następnego stanu
	  					// Jeżeli nie, należy zignorować ramkę, bo nawet gdyby dalsza analiza wykazałaby, że jest poprawna, nie jest ona przeznaczona dla tego urządzenia
	  				case CHECK_RECEIVER_SENDER:
	  					;

	  					// Porównanie odbiorcy
	  					if (strncmp(&frame[3], device_address, 3) == 0) {

	  						// Kopiowanie nadawcy
	  						strncpy(source_address, frame, 3);
	  						frame_state = CHECK_COMMAND_LENGTH;
	  					}
	  					else {
	  						reset_frame_state();
	  					}

	  					break;

	  					// Stan odpowiada za sprawdzanie długości komendy, czyli ilości znaków, które są w polu 'dane' ramki i które mają być traktowane jako polecenie dla programu
	  					// Najpierw należy wyodrębnić 3 znaki dłguości komendy do osobnej tablicy,
	  					// Następnie należy sprawdzić, czy znaki są cyframi
	  					// Jeżeli nie, należy zwrócić komunikat o złej długości komendy
	  					// Jeżeli tak, należy sprawdzić:
	  					// Czy podana dłguość komendy nie jest większa niż maksymalna ilość znaków w ramce 'dane' (256)
	  					// Czy ilość znaków w polu 'dane' nie wynosi 0 (wtedy ramka jest 'pusta', bo nie ma danych do przeanalizowania)
	  					// Jeżeli długość komendy się zgadza, należy przejść do następnego stanu
	  				case CHECK_COMMAND_LENGTH:
	  					;

	  					// Kopiowanie znaków długości komendy do tablicy pomocniczej
	  					strncpy(temp, &frame[6], 3);

	  					// Sprawdzanie, czy znaki są cyframi
	  					if (is_string_alphanumeric(temp, 3) == 0) {

	  						// nieprawidłowa długość komendy
	  						send_answer_message("WRLEN");
	  						reset_frame_state();
	  						break;
	  					}

	  					// Zamiana z tablicy charów (stringa) na int
	  					command_length = atoi(temp);

						// Sprawdzanie, czy długość komendy nie posiada za dużej wartości (niż zadeklarowana ilość danych w ramce dane w protokole na papierze)
						// Oraz czy ramka nie przyszła pusta (wartość int długości komendy > 0 a ilość danych 0)
						if (command_length <= 256 && frame_length - frame_min_length == command_length) {
							//poprawne
							frame_state = CHECK_CHECKSUM;
							// Sprawdzanie, czy ramka nie przyszła pusta
						} else if (frame_length - frame_min_length == 0) {
							reset_frame_state();

							//pusta ramka
							send_answer_message("WRFRM");
						}
						// Zwracanie błędu, że długość komendy jest zła
						else {
							reset_frame_state();
							//zła długość komendy
							send_answer_message("WRLEN");
						}
						break;

						// Stan odpowiada za porównanie otrzymanej sumy kontrolnej z wyliczoną na podstawie otrzymanych danych
						// Najpierw należy wydzielić znaki sumy kontrolnej do osobnej tablicy, a następnie sprawdzić, czy są cyframi
						// Jeżeli nie, należy wysłać komunikat zwrotny o złej sumie kontrolnej
						// Jeżeli tak, należy zamienić znaki na wartość int
						// Następnie należy wyliczyć wartość dziesiętną wszystkich znaków z pola 'dane'
						// Po zsumowaniu wartości, należy podzielić z resztą (modulo '%') sumę przez z góry ustalony dzielnik 'checksum_div'
						// Ostatecznie należy porównać obie sumy kontrolne
						// Jeżeli obliczona suma różni się od otrzymanej, należy wysłać odpowiedni komunikat i wrócić do stanu FIND_FRAME_START
						// Jeżeli sumy się zgadzają, należy przejść dalej
	  				case CHECK_CHECKSUM:
	  					;

	  					uint8_t checksumLength = 3;

	  					// Definiowanie miejsca, w którym zaczyna się w ramce suma kontrolna (gdy suma jest po danych)
	  					uint16_t checksumStart = frame_length - checksumLength;
	  					uint16_t checksum;

	  					// Kopiowanie sumy kontrolnej do tablicy pomocniczej
	  					strncpy(temp, &frame[checksumStart], 3);

	  					// Sprawdzanie, czy znaki sumy kontrolnej są cyframi
	  					if (is_string_alphanumeric(temp, 3) == 0) {

	  						// nieprawidłowa suma kontrolna
	  						send_answer_message("WRCHSUM");
	  						reset_frame_state();
	  						break;
	  					}
	  					checksum = atoi(temp);

	  					uint16_t dataSum = 0;

	  					// Kolekcjonowanie wartości liczbowej danych
	  					for (int i = 9; i < checksumStart; i++)
	  						dataSum += frame[i];

	  					// Obliczanie reszty z sumy kontrolnej
	  					uint8_t readChecksum = dataSum % checksum_div;

	  					// Porównanie sumy kontrolnej
	  					if (checksum == readChecksum) {
	  						frame_state = ANALYZE_COMMAND;
	  					}
	  					else {
	  						send_answer_message("WRCHSUM");
	  						reset_frame_state();
	  					}
	  					break;

	  					// Stan ten odpowiada za przeanalizowanie komendy, czyli znaków z pola 'dane', które mają być traktowane jako polecenie
	  					// Tutaj analiza odbywa się w zależności od tego, co zaprojektowano w protkole
	  				case ANALYZE_COMMAND:
	  					char command[256];
	  					uint8_t acctual_command = frame_length - frame_min_length;
	  					strncpy(command, &frame[9], acctual_command);

	  					//Reset urządzenia
	  					if (strcmp(command, "RST") == 0) {
	  						// Wykonanie resetu
	  						QMC_Reset(&sensor);
	  						send_answer_message("RESET");
	  						reset_frame_state();
	  						break;
	  					}

	  					//Standby
	  					else if (strcmp(command, "STANDBY") == 0){
	  						QMC_Standby(&sensor);
	  						send_answer_message("ZASTOPOWANO");
	  						reset_frame_state();
	  						break;
	  					}

	  					// Ustawienie i włączenie pobioru danych w danym interwale czasowym
	  					else if (strncmp(command, "SETINTERVAL[", 12) == 0) {
	  						char *delayPtr = command + 12; // wskaźnik na fragment po "SETINTERVAL["
	  						char *endBracket = strchr(delayPtr, ']');
	  						if (endBracket != NULL) {
	  							*endBracket = '\0'; // Zakończ ciąg
	  							int Hz = atoi(delayPtr); // Konwersja na int
	  							if (Hz == 200 || Hz == 100 || Hz == 50 || Hz == 10) {
	  								QMC_init(&sensor, &hi2c1, Hz);
	  								send_answer_message("INTERVAL=%d", Hz);
	  								reset_frame_state();
	  								break;
	  							} else {
	  								send_answer_message("WPISANO_ZLY_INTERWAL");
	  								reset_frame_state();
	  								break;
	  							}
	  						}
	  					}



	  					// Aktualne dane pobiór
	  					else if (strcmp(command, "GET") == 0)
	  					{
	  						QMC_read(&sensor, &x, &y, &z);
	  						send_answer_message("X:%d,Y:%d,Z:%d\n", x, y, z);
	  						reset_frame_state();
	  						break;
	  					}

			  // Pobór kąta
			 /* else if (strcmp(command, "GETANGLE") == 0)
			  {
				  QMC_read(&sensor, &angle);
				  send_answer_message("ANGLE: %d\n", angle);
				  reset_frame_state();
				  break;
			  }
			  else if (strcmp(command, "CFG") == 0)
			  {
				  if (Hz == 200 || Hz == 100 || Hz == 50 || Hz == 10)
				  {
					  send_answer_message("Hz: %d", Hz);
					  reset_frame_state();
					  break;
				  }
				  else
				  {
					  send_answer_message("NIE_USTAWIONO_INTERWALU");
					  reset_frame_state();
					  break;
				  }
			  }*/

	  				}

	  			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	  }
  /* USER CODE END 3 */
	}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
