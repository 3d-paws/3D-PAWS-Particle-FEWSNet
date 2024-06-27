/*
 * ======================================================================================================================
 *  Rain.h - Rain Functions
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 *  Rain Gauges
 * ======================================================================================================================
 */
#define RAINGAUGE1_IRQ_PIN  A3
#define RAINGAUGE2_IRQ_PIN  A2

/*
 * ======================================================================================================================
 *  Rain Gauge 1 - Optipolar Hall Effect Sensor SS451A
 * ======================================================================================================================
 */
volatile unsigned int raingauge1_interrupt_count;
uint64_t raingauge1_interrupt_stime; // Send Time
uint64_t raingauge1_interrupt_ltime; // Last Time
uint64_t raingauge1_interrupt_toi;   // Time of Interrupt

/*
 * ======================================================================================================================
 *  raingauge1_interrupt_handler() - This function is called whenever a magnet/interrupt is detected by the arduino
 * ======================================================================================================================
 */
void raingauge1_interrupt_handler()
{
  if ((System.millis() - raingauge1_interrupt_ltime) > 500) { // Count tip if a half second has gone by since last interrupt
    digitalWrite(LED_PIN, HIGH);
    raingauge1_interrupt_ltime = System.millis();
    raingauge1_interrupt_count++;
    TurnLedOff = true;
  }   
}

/*
 * ======================================================================================================================
 *  Rain Gauge 2 - Optipolar Hall Effect Sensor SS451A
 * ======================================================================================================================
 */
volatile unsigned int raingauge2_interrupt_count;
uint64_t raingauge2_interrupt_stime; // Send Time
uint64_t raingauge2_interrupt_ltime; // Last Time
uint64_t raingauge2_interrupt_toi;   // Time of Interrupt

/*
 * ======================================================================================================================
 *  raingauge2_interrupt_handler() - This function is called whenever a magnet/interrupt is detected by the arduino
 * ======================================================================================================================
 */
void raingauge2_interrupt_handler()
{
  if ((System.millis() - raingauge2_interrupt_ltime) > 500) { // Count tip if a half second has gone by since last interrupt
    digitalWrite(LED_PIN, HIGH);
    raingauge2_interrupt_ltime = System.millis();
    raingauge2_interrupt_count++;
    TurnLedOff = true;
  }   
}
