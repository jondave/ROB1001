int x=0;
void setup() 
{
    Serial.begin( 115200 );  // fast
    Serial.write( "RS" );    // restart!
}
void loop() 
{
  delay(2000);
  x=x+1;
  Serial.print( x ); 
}
