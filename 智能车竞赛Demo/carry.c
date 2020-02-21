void test_adc(void);
void test_sf(void);
void test_dc(void);
void go_line(void);
void go_line2(void);
void go2(int x,int y);
void yssb(void);

int pcs;
int dhl,dhr,f1,f2,f3,f4,f5,fl,fr,gdl=320,gdr=400,gd1=280,gd2=210,gd3=290,gd4=320,gd5=420;
int gd_he=700,gd_la=552,gd_lv=570,gd_hu=460,gd_ba=390,ys=0;
int xx,yy;
float tt;

int main(void)
{

  pcs=selector();
  if(pcs==0) test_adc();        
  else if(pcs==1) test_sf();   
  else if(pcs==2) test_dc();   
  else if(pcs==3) {wait(2);while(1) { go_line(); }}  
  wait(2);
  seconds(0);
  while(seconds(1)<2||getadc(7)<gdr){go_line();}
  while(getadc(5)<gd5){go_line();}
  go(200,200);while(getadc(6)<gdr){;}wait(0.1);while(getadc(6)>gdr){;}
  go(200,-200);while(getadc(5)<gd5){;}
  go(200,-200);while(getadc(3)<gd3){;}

  while(getadc(5)<gd5){go_line();}
  tt=seconds(1)+1;
  while(seconds(1)<tt){go_line2();}
  
  go2(0,0);while(1){;}  
} 
void go2(int x,int y)
{
  if(x!=xx) motor(1,x); xx=x;
  if(y!=yy) motor(2,y); yy=y;
}
void go_line(void)
{
 dhl=getadc(7);dhr=getadc(6);
 f1=getadc(1);f2=getadc(2);f3=getadc(3);f4=getadc(4);f5=getadc(5);
 
  
 if(f3>gd3)
 {
  if(f4>gd4)go2(170,130);  
  else if(f2>gd2)go2(130,170); 
  else go2(150,150);
  }
 else if(f4>gd4&&f2<gd2)go2(200,100);  
 else if(f2>gd2&&f4<gd4)go2(100,200); 
 else go2(150,150);
}
void go_line2(void)
{
 dhl=getadc(7);dhr=getadc(6);
 f1=getadc(1);f2=getadc(2);f3=getadc(3);f4=getadc(4);f5=getadc(5);
 
 if(f5>gd5)go2(200,0); 
 else if(f1>gd1)go2(0,200); 
 
 else if(f3>gd3)
 {
  if(f4>gd4)go2(170,130);  
  else if(f2>gd2)go2(130,170); 
  else go2(150,150);
  }
 else if(f4>gd4&&f2<gd2)go2(200,100);  
 else if(f2>gd2&&f4<gd4)go2(100,200); 
 else go2(150,150);
}
void yssb(void)
{
 fl=getadc(8);fr=getadc(9);
 
 if(fl<gd_he+20&&fl>gd_he-20)ys=1;
 else if(fl<gd_la+20&&fl>gd_la-20)ys=2;
 else if(fl<gd_lv+20&&fl>gd_lv-20)ys=3;
 else if(fl<gd_hu+20&&fl>gd_hu-20)ys=4;
 else if(fl<gd_ba+20&&fl>gd_ba-20)ys=5;
 else ys=0;
}

void test_adc(void)
{
  lcd_on();
  while(1)
  { dhl=getadc(7);dhr=getadc(6);
    fl=getadc(8);fr=getadc(9);
    f1=getadc(1);f2=getadc(2);f3=getadc(3);f4=getadc(4);f5=getadc(5);
    yssb();
    cls();
    printf("F1=%3d   F2=%3d",f1,f2);
    locate(2,5);
    printf("F3=%3d",f3);
    locate(3,1);
    printf("F4=%3d   F5=%3d",f4,f5);
    locate(4,1);
    printf("L=%3d R=%3d ",dhl,dhr);
    locate(5,1);
    printf("FL=%3d FR=%3d ",fl,fr);
    locate(6,1);
    printf("YS=%d",ys);
    wait(0.1);
  }
  lcd_off();
}


void test_sf(void)
{
  lcd_on();
  while(1)
  { 
   servo(1,110);wait(1);
   servo(1,70);wait(1);
  }
  
}  

void test_dc(void)
{
  lcd_on();
  wait(1);
  go2(150,150);    
  wait(10);
  go2(-150,-150);    
  wait(1);
  go2(150,0); 
  wait(1);
  go2(0,150);    
  wait(1);    
  go2(0,0); 
  sound(2000,500);while(1){;}
}























