
#define DISP_LEN 4

#define BAR0_SEG 1
#define BAR1_SEG 2
#define NUM0_SEG 3
#define NUM1_SEG 4

void Send_7219(int8_t rg, int8_t dt);
void Clear_7219(void);
void Set_Bar(uint8_t dt);
void Disp_Bars(uint8_t dt);
void Disp_Num_Seg(uint8_t seg, uint8_t num, uint8_t dot);
void Disp_Num(int8_t num, uint8_t dot);
