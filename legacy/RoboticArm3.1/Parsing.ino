int ricavaComando(){
  switch(s[0]){
    case('M'):
     switch(s[1]){
      case('0'):
        if(s[2] == '3')
          return(4);
        else
          return(-1);
        break;
      case('3'):
        if(s[2] == '0' )
          return(5);
        else
          return(-1);
        break;
      default:
        return(-1);
        break;
     }
   case('G'):
     switch(s[1]){
      case('9'):
        switch(s[2]){
          case('0'):
            return(2);
            break;
          case('1'):
            return(3);
            break;
          default:
            return(-1);
            break;
        }
        break;
      case('0'):
        return(0);
        break;
      case('1'):
        switch(s[2]){
          case('6'):
            return(7); //passa a coord polari
            break;
          case('5'):
            return(8); //passa a coord cartesiane
            break;
          default:
            return(1); //movimento controllato
            break;
        }
        break;
      default:
        return(-1);
        break;       
     }
   case('A'):
    return(6);
    break;
  case('I'):
    return(9);
    break;
  }
}
void dividiStringa(){
  int i=2;
  int j=0;
  indx=indy=indz=indf=-1;
  while(s[i]!='\0' && s[i]!='\n' && s[i]!='G' && s[i]!= 'M'){
    switch(s[i]){
      case('X'):
        indx=i;
        i++;
        while((s[i]>='0' && s[i]<='9')||(s[i]=='-'&& i==indx+1)||(s[i]=='.')){
           if(j>DIM_BUFFER_SX)
              Serial.println("sx e' andato in overfow");
           sx[j]=s[i];
           i++;
           j++;
        }
        sx[j]='\0';
        j=0;
        break;
      case('Y'):
        indy=i;
        i++;
        while((s[i]>='0' && s[i]<='9')||(s[i]=='-'&& i==indy+1)||(s[i]=='.')){
           if(j>DIM_BUFFER_SY)
             Serial.println("sy e' andato in overfow");
           sy[j]=s[i];
           i++;
           j++;
           //Serial.print(sy[j]);
        }
        //Serial.print('\n');
        sy[j]='\0';
        j=0;
        break;
      case('Z'):
        indz=i;
        i++;
        while((s[i]>='0' && s[i]<='9')||(s[i]=='-'&& i==indz+1)||(s[i]=='.')){
           if(j>DIM_BUFFER_SZ)
              Serial.println("sz e' andato in overfow");
           sz[j]=s[i];
           i++;
           j++;
           //Serial.print(sz[j]);
        }
        //Serial.print('\n');
        sz[j]='\0';
        j=0;
        break;
      case('F'):
        indf=i;
        i++;
        while(s[i]>='0' && s[i]<='9'){
           sf[j]=s[i];
           i++;
           j++;
        }
        sf[j]='\0';
        j=0;
        break;
      default:
        i++; 
    }
  }
}

void toCapital(){
  //Serial.println("toCapital");
  int i=0;
  while(s[i]!='\n' && s[i]!='\0'){
    if(s[i]<='z' && s[i]>='a')
      s[i]=s[i]-'a'+'A';
    i++;
  }
}
void removeSpaces(){
  //Serial.println("removeSpaces");
  int k=0;
  boolean StillSpace=1;
  for (int i=0; s[i]!='\0'&&s[i]!='\n'; i++){
    if (s[i]!=' '){
      s[k]=s[i];
      k++;
    }
  }
  s[k]='\0';  
}
boolean parse(){
  toCapital();
  removeSpaces();
  int command = ricavaComando();
  switch(command){
    case(0): //g0, muoviti a vel massima
      dividiStringa();
      if (indx != -1)
        param[0] = atof(sx) + coord_rel * ((coord_now[0] * (!coord_cilind)) + (coord_now_cilin[0] * coord_cilind)); //per avere un integratore anzichè cose strane dovremi mettere invece di coord_now posit_fin
      else
        param[0] = (coord_now[0] * (!coord_cilind)) + (coord_now_cilin[0] * coord_cilind);
      if (indy != -1)
        param[1] = atof(sy) + coord_rel * ((coord_now[1] * (!coord_cilind)) + (coord_now_cilin[1] * coord_cilind));
      else
        param[1] = (coord_now[1] * (!coord_cilind)) + (coord_now_cilin[1] * coord_cilind);
      if (indz != -1)
        param[2] = atof(sz) + coord_rel * ((coord_now[2] * (!coord_cilind)) + (coord_now_cilin[2] * coord_cilind));
      else
        param[2] = (coord_now[2] * (!coord_cilind)) + (coord_now_cilin[2] * coord_cilind);
      param[3] = VEL_MAX;
      param[4] = status_em;
      break;
    case(1): //g1, muoviti a velocità controllata
      dividiStringa();
      if (indx != -1)
        param[0] = atof(sx) + coord_rel * ((coord_now[0] * (!coord_cilind)) + (coord_now_cilin[0] * coord_cilind));
      else
        param[0] = (coord_now[0] * (!coord_cilind)) + (coord_now_cilin[0] * coord_cilind);
      if (indy != -1)
        param[1] = atof(sy) + coord_rel * ((coord_now[1] * (!coord_cilind)) + (coord_now_cilin[1] * coord_cilind));
      else
        param[1] = (coord_now[1] * (!coord_cilind)) + (coord_now_cilin[1] * coord_cilind);
      if (indz != -1)
        param[2] = atof(sz) + coord_rel * ((coord_now[2] * (!coord_cilind)) + (coord_now_cilin[2] * coord_cilind));
      else
        param[2] = (coord_now[2] * (!coord_cilind)) + (coord_now_cilin[2] * coord_cilind);
      if (indf != -1){
        param[3] = atof(sf);
        feedvel = param[3];
      }
      else 
        param[3] = feedvel;
      param[4] = status_em;
      break;
    case(2): //g90, passa in coordinate assolute
      Serial.println("coord ass");
      coord_rel = 0;
      return(0);
      break;
    case(3): //g91, passa in coordinate relative
      Serial.println("coord rel");
      coord_rel=1;
      return(0);
      break;
    case(4): //m03, accendi elettromagnete
      status_em=1;
      return(0);
      break;
    case(5): //m30, spegni EM
      status_em=0;
      return(0);
      break;
    case(6): //A, chiede l' angolo del primo joint
      Serial.println(angles[0]);
      return(0);
      break;
    case(7):
      Serial.println("coord cilindriche");
      coord_cilind=1;
      return(0);
      break;
    case(8):
      Serial.println("coord cartesiane");
      coord_cilind=0;
      return(0);
      break; 
    case(9): //I, chiede se il robot è 
      if(idle)
        Serial.println(idle);
      else
        Serial.println(idle);
      return(0);
      break;
    default: //se la stringa è sbagliata mando la posizione attuale
      param[0] = (coord_now[0] * (!coord_cilind)) + (coord_now_cilin[0] * coord_cilind);
      param[1] = (coord_now[1] * (!coord_cilind)) + (coord_now_cilin[1] * coord_cilind);
      param[2] = (coord_now[2] * (!coord_cilind)) + (coord_now_cilin[2] * coord_cilind);
      param[3] = VEL_MAX;
      param[4] = status_em;
      break;
  }
//  for(int i=0; i<5; i++){
//    Serial.print("Parametro ");
//    Serial.print(i);
//    Serial.print(" :  ");
//    Serial.println(param[i]);
//    if(i<3){
//      Serial.print("coord_now");
//      Serial.print(i);
//      Serial.print(" :  ");
//      Serial.println(coord_now[i]);
//    }
//  }
  chr = NULL;
  //Serial.print("ricevuto : ");
  //Serial.println(s);
  s[0] = '\0';
  if(glitch_flag){
      glitch_flag = 0;
   //   Serial.println("glitch risolto");
  }
  Serial.println("ok");
  return(1);
}

