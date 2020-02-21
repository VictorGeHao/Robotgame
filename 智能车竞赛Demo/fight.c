void test_adc(void);
void cgq(void);
void test_dc(void);
void gj(void);
void tui(void);
void huizhong(void);

int irf1,irf2,irf3,irf4,irf5,irr,irl,irb,irx,iry;
int dhfl,dhfr,dhbr,dhbl;
int dhlr;
int pcs;
//地灰     白        黑        出
int gdmin=310,gdmax=500,gdout=540; 
//测距参数，区分有无障碍物
int n=120,ir_max=300,za_y;              
int ii,ix=10000;
float tt;


int main(void)
{
   pcs=selector();
   if(pcs==0)  test_adc();
   else if(pcs==1)  test_dc();
   else if(pcs==2){while(1){cgq();if(irx>n&&za_y<1000)za_y++;else za_y=0;if(za_y>50){gj();}else {go(0,0);}}} //测攻击
   else if(pcs==3){while(1){cgq();huizhong();}}//测回中
   else if(pcs==4){wait(1);go(-500,-500);wait(0.7);go(0,0);while(1){;}}//测上坡
   else if(pcs==5){wait(1);while(1){cgq();if(irx>n&&za_y<1000)za_y++;else za_y=0;if(za_y>50)tui();else go(0,0);}}//测前推
   
 //上坡
{
  go(0,0);wait(1);
  go(-500,-500);wait(0.7);
  go(0,0);wait(0.01);
  go(0,500);wait(0.6);
  go(500,500);wait(0.5);
  go(0,0);wait(0.01);
}
  while(1)
 {
  cgq();
  if(irx>n&&za_y<1000)za_y++;else za_y=0;
  if(za_y>10){gj();}
  else {huizhong();}
 }
   
}

void cgq(void)
{
    irf1=geteadc(1);irf2=geteadc(2);irf3=geteadc(3);irf4=geteadc(4);irf5=geteadc(5);
    irr=geteadc(6);irb=getadc(2);irl=getadc(4);
    
  irx=n; iry=0;
  if(irf1>irx) {irx=irf1;iry=1;} 
  if(irf2>irx) {irx=irf2;iry=2;} 
  if(irf3>irx) {irx=irf3;iry=3;} 
  if(irf4>irx) {irx=irf4;iry=4;} 
  if(irf5>irx) {irx=irf5;iry=5;} 
  if(irr>irx) {irx=irr;iry=6;} 
  if(irb>irx) {irx=irb;iry=7;} 
  if(irl>irx) {irx=irl;iry=8;} 

    
    
    dhfl=geteadc(7)-100;
    dhfr=geteadc(8)-250;
    dhbr=getadc(1)-25;
    dhbl=getadc(3)-100;
}
 
//--   ____3____
//--   __2___4__
//--   1_______5

//-- 8           6

//--       7
//攻击
void gj(void)
{
 if(iry==3)  
 {if(dhfl>gdout){go(-1000,-700);wait(0.2);}
  else if(dhfr>gdout){go(-700,-1000);wait(0.2);}
  else if(dhfl>gdmax&&dhfr>gdmax){if(dhfl>dhfr)go(300,400);else go(400,300);}
  else if(dhfl>dhfr)go(500,600);
  else go(600,500);
  } 
 else if(iry==2)
 {if(dhfl>gdout){go(-1000,-700);wait(0.2);}
  else if(dhfr>gdout){go(-700,-1000);wait(0.2);}
  else if(dhfl>gdmax&&dhfr>gdmax){if(dhfl>dhfr)go(300,400);else go(400,300);}
  else if(dhfl>dhfr)go(500,600);
  else go(600,500);
  } 
 else if(iry==4)
 {if(dhfl>gdout){go(-1000,-700);wait(0.2);}
  else if(dhfr>gdout){go(-700,-1000);wait(0.2);}
  else if(dhfl>gdmax&&dhfr>gdmax){if(dhfl>dhfr)go(300,400);else go(400,300);}
  else if(dhfl>dhfr)go(500,600);
  else go(600,500);
  } 
 else if(iry==1){{go(100,500);}} 
 else if(iry==5){{go(500,100);}}
 else if(iry==8) {go(-300,300);tt=seconds(1)+0.5;while(seconds(1)<tt&&(geteadc(2)<n&&geteadc(3)<n)){;}}
 else if(iry==6) {go(300,-300);tt=seconds(1)+0.5;while(seconds(1)<tt&&(geteadc(3)<n&&geteadc(4)<n)){;}}
 else if(iry==7) {go(300,-300);tt=seconds(1)+0.7;while(seconds(1)<tt&&(geteadc(3)<n&&geteadc(4)<n)){;}}
 else {go(0,0);} 
 }

void tui(void)
{
 if(iry==3){go(200,200);} 
 else if(iry==2){go(100,200);} 
 else if(iry==4){go(200,100);} 
 else if(iry==1){go(0,200);} 
 else if(iry==5){go(200,0);}
 else if(iry==8){go(-200,200);tt=seconds(1)+0.5;while(seconds(1)<tt&&(geteadc(2)<n&&geteadc(3)<n)){;}}
 else if(iry==6){go(200,-200);tt=seconds(1)+0.5;while(seconds(1)<tt&&(geteadc(3)<n&&geteadc(4)<n)){;}}
 else if(iry==7){go(200,-200);tt=seconds(1)+0.7;while(seconds(1)<tt&&(geteadc(3)<n&&geteadc(4)<n)){;}}
 else {go(0,0);} 
 }

//回中
void huizhong(void)
{ 
  if(dhfl<gdmin||dhfr<gdmin)
  {go(200,200);;wait(0.2);
    while(irx<n)
   {cgq();{go(150,-150);}}
  }
  else
  {dhlr=dhbl-dhbr;
    if(dhlr<-10)
    {
      go(-200,200);
    }
    else if(dhlr>10)
    {
    go(200,-200);
    }
    else {go(300,300); }
    }
}
void test_adc(void)
{
  sound(500,500);
  
  while(1)
  {
    cgq();
    cls();
    locate(1,4);
    printf("TEST_ADC");
    locate(2,1);
    printf("f1=%3d f2=%3d",irf1,irf2);
    locate(3,1);
    printf("f3=%3d f4=%3d",irf3,irf4);
    locate(4,1);
    printf("f5=%3d r =%3d",irf5,irr);
    locate(5,1);
    printf("b =%3d l =%3d",irb,irl);
    locate(6,1);
    printf("gdfl=%3d gdfr=%3d",dhfl,dhfr);
    locate(7,1);
    printf("gdbl=%3d gdbr=%3d",dhbl,dhbr);
    locate(8,1);
    printf("irx=%2d iry=%d",irx,iry);
    wait(0.05);
  }
}

void test_dc(void)
{
   go(500,500);
   wait(1);
   go(-500,-500);
   wait(1);
   go(500,0);
   wait(1);
   go(0,500);
   wait(1);
   go(0,0);
}

