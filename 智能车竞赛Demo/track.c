void test_adc(void);
void test_dc(void);
void test_dk(void);
void go_f(void);
void go_b(void); 
void go_T_fr(void);
void go_T_fl(void);
void go_T_br(void);
void go_fp(void);
void go_bp(void);
void go_T_bl(void);
void gof(int x,int y);
void gob(int x,int y);

int ff1,ff2,ff3,ff4,ff5,ff6,ff7,bb1,bb2,bb3,bb4,bb5,bb6,bb7,fcz,bcz; 
int shuzi1,shuzi2,shuzi3,shuzi4;
int pcs;
int ii;
float tt;
//������ʱ�0-2
float k=1;

int gdf1=450,gdf2=450,gdf3=450,gdf4=450,gdf5=450,gdf6=450,gdf7=450;
int gdb1=450,gdb2=450,gdb3=450,gdb4=450,gdb5=450,gdb6=450,gdb7=450;
//�ػ�ֵgd���ڰ������м��ֵ,��ֵ��600���ң���Ҫ�������λ��������
int gd=400;

void gof(int x,int y)
{
  motor(1,x*k); 
  motor(2,y*k); 
}
void gob(int x,int y)
{
  motor(2,-x*k); 
  motor(1,-y*k); 
}


int main(void)
{
  //����ѡ��  
  pcs=selector();
  if(pcs==0) test_adc();        //����ǰ��Ҷȴ�����  
  else if(pcs==1) test_dc();    //���Ե��
  else if(pcs==2) 
  {
   wait(2);tt=seconds(1)+1;
   while(seconds(1)<tt){go_f();}gof(0,0);while(1){;}
   }
  else if(pcs==3) 
  {
   wait(2);tt=seconds(1)+1;
   while(seconds(1)<tt){go_b();}gob(0,0);while(1){;}
   }
//��
//ǰ��2�����ߵ�����������תײ����
 else if(pcs==4) 
  {
   wait(2);
   gof(200,200);wait(0.1);//����
//��ǰ������ߵػ�������
   while(geteadc(1)>gd){go_f();}//��û�п�����һ������ʱ��ǰѰ����
   gof(0,0);wait(2); //�����ֽ⶯�����ĸ������������ͣ������һ��
   tt=seconds(1)+0.2;while(geteadc(1)<gd||seconds(1)<tt){go_f();}//�������߻���С��0.2��ǰѰ���ߣ�������
   while(geteadc(1)>gd){go_f();}//��û�п����ڶ�������ʱ��ǰѰ����
   tt=seconds(1)+0.2;while(geteadc(1)<gd||seconds(1)<tt){go_f();}//�������߻���С��0.2��ǰѰ���ߣ�������
   while(geteadc(1)>gd){go_f();}//��û�п�������������ʱ��ǰѰ����
   k=0.5;//�������߼��٣�������ʼ���
   tt=seconds(1)+0.1;while(seconds(1)<tt){go_f();}//��������ʹ������ǰ�ֹ����ߣ�׼��ת��
   k=1;//�ָ��������
   go_T_fr();//��ת��
   tt=seconds(1)+0.1;while(seconds(1)<tt){go_f();}//׼��ײ���㣬ת��ڵ�������볤���Ӵ�ʱ��
   gof(0,0);wait(2); 
   go_fp();
   gof(0,0);while(1){;}
   }
  wait(2);
  seconds(0);
  gof(200,200);wait(0.1);//����
  while(getadc(1)<200){go_f();}
  
  gof(0,0);while(1){;}

}

void go_f(void)
{ 
  test_dk();
  if(ff4<gd)     
  {
   if(ff5<gd)     gof(350,250);
   else if(ff3<gd)gof(250,350);
   else           gof(300,300);
   }
  else if(ff5<gd)
  {
   if(ff6<gd)gof(300,150);
   else      gof(300,200);
   }
  else if(ff6<gd)
  {
   if(ff7<gd)gof(300, 50);
   else      gof(300,100);
   }
  else if(ff3<gd)  
  {
   if(ff2<gd)gof(150,300);
   else      gof(200,300);
   }
  else if(ff2<gd)  
  {
   if(ff1<gd)gof( 50,300);
   else      gof(100,300);
   }
  else {gof(100,100);}
}

void go_b(void)
{
  test_dk();
  if(bb4<gd)     
  {
   if(bb5<gd)     gob(350,250);
   else if(bb3<gd)gob(250,350);
   else           gob(300,300);
   }
  else if(bb5<gd)
  {
   if(bb6<gd)gob(300,150);
   else      gob(300,200);
   }
  else if(bb6<gd)
  {
   if(bb7<gd)gob(300, 50);
   else      gob(300,100);
   }
  else if(bb3<gd)  
  {
   if(bb2<gd)gob(150,300);
   else      gob(200,300);
   }
  else if(bb2<gd)  
  {
   if(bb1<gd)gob( 50,300);
   else      gob(100,300);
   }
  else {gob(100,100);}
}

void go_T_fr(void)
{
 gof(150,-250);while(geteadc(7)>gd){;}
 gof(150,-250);while(geteadc(4)>gd&&geteadc(5)>gd){;}
}
void go_T_fl(void)
{
 gof(-250,150);while(geteadc(1)>gd){;}
 gof(-250,150);while(geteadc(4)>gd&&geteadc(3)>gd){;}
}
void go_T_br(void)
{
 gob(150,-250);while(geteadc(15)>gd){;}
 gob(150,-250);while(geteadc(12)>gd&&geteadc(13)>gd){;}
}
void go_T_bl(void)
{
 gob(-250,150);while(geteadc(9)>gd){;}
 gob(-250,150);while(geteadc(12)>gd&&geteadc(11)>gd){;}
}

void go_fp(void)
{
  k=0.5;
  while(ii<100){if(getadc(1)>200)ii=ii++;else ii=0;go_f();}ii=0;
  tt=seconds(1)+0.2;while(seconds(1)<tt){go_f(); }  gob(150,150);wait(0.1);
  tt=seconds(1)+0.2;while(seconds(1)<tt){go_b(); }
  k=1;
  //gof(0,0);wait(2); 
}
void go_bp(void)
{
  k=0.5;
  while(ii<100){if(getadc(2)>200)ii=ii++;else ii=0;go_b();}ii=0;
  tt=seconds(1)+0.2;while(seconds(1)<tt){go_b(); }  gof(150,150);wait(0.1);
  tt=seconds(1)+0.2;while(seconds(1)<tt){go_f(); }
  k=1;
  //gob(0,0);wait(2); 
}

void test_dk(void)
{
  fcz=getadc(1);
  bcz=getadc(2);
  ff1=geteadc(1); ff2=geteadc(2); ff3=geteadc(3);
  ff4=geteadc(4);
  ff5=geteadc(5); ff6=geteadc(6); ff7=geteadc(7);
  bb1=geteadc(9);bb2=geteadc(10);bb3=geteadc(11);
  bb4=geteadc(12);
  bb5=geteadc(13);bb6=geteadc(14);bb7=geteadc(15);
  shuzi1=getport(7);
  shuzi2=getport(8);
  shuzi3=getport(11);
  shuzi4=getport(12);
}

  
//���Ը���ģ�������
void test_adc(void)
{
  lcd_on();
  while(1)
  { 
    test_dk();
    cls();
    printf("F1=%3d F2=%3d F3=%3d",ff1,ff2,ff3);
    locate(2,1);
    printf("F4=%3d FCZ=%3d",ff4,fcz);
    locate(3,1);
    printf("F5=%3d F6=%3d F7=%3d",ff5,ff6,ff7);
    locate(4,1);
    printf("B1=%3d B2=%3d B3=%3d",bb1,bb2,bb3);
    locate(5,1);
    printf("B4=%3d BCZ=%3d",bb4,bcz);
    locate(6,1);
    printf("B5=%3d B6=%3d B7=%3d",bb5,bb6,bb7);
    wait(0.1);
    printf("SZ1=%1d,SZ2=%1d,SZ3=%1d,SZ4=%1d",shuzi1,shuzi2,shuzi3,shuzi4);
    wait(0.1);
  }
  lcd_off();
}

//�������
void test_dc(void)
{while(trigger()==1){;}sound(500,500);
 while(1)
 {pcs=selector();
  locate(1,1);printf("pcs=(%2d)",pcs);wait(0.05);
  cls();
  if(pcs==0&&trigger()==1) {sound(500,500);go(100,100);wait(1);go(-100,-100);wait(1);go(0,0);sound(500,500);}
  else if(pcs==1&&trigger()==1) {sound(500,500);gof(100,0);while(trigger()==0){;}gof(0,0);sound(500,500);}
  else if(pcs==2&&trigger()==1) {sound(500,500);gof(0,100);while(trigger()==0){;}gof(0,0);sound(500,500);}
  else if(pcs==3&&trigger()==1) {sound(500,500);gob(100,0);while(trigger()==0){;}gob(0,0);sound(500,500);}
  else if(pcs==4&&trigger()==1) {sound(500,500);gob(0,100);while(trigger()==0){;}gob(0,0);sound(500,500);}
 }

}

























