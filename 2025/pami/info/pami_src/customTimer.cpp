#include <Arduino.h>
#include "customTimer.h"
#include "config.h"
#include "led.h"
#include "motor.h"

unsigned long tempsDepartMs;    // Pour enregistrer le temps auquel démarre le combat

void CustomTimerInit()
{
  tempsDepartMs = millis();

  //Serial.print("Il est : ");
  //Serial.println(tempsDepartMs);

  // Init timer ITimer1
  ITimer1.init();
  // Interval in unsigned long millisecs
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, CustomTimerCheckTime);
}

void CustomTimerCheckTime()
{
  // On lit le temps actuel
  unsigned long tempsCourantMs = millis();
  // On le compare avec le temps au début du match
  if ((tempsCourantMs - tempsDepartMs) > DUREE_MATCH_MS)
  {
    Serial.print("Il est : ");
    Serial.println(tempsCourantMs);
    Serial.print("Fin!");
    // Si la différence des deux fait plus que le DUREE_COMBAT_MS alors on déclenche la fonction finDeMatch
    FinDeMatch();
  }
}

// Fonction de fin de combat, qui arrête le robot
void FinDeMatch()
{
  // On arrête les moteurs
  MotorLeftSetSpeed(0);
  MotorRightSetSpeed(0);
  MotorStop();

  while (1)
  {
    // On boucle à rien faire dans une boucle sans fin
    // c'est pas optimal, de déclencher une boucle sans fin dans une interruption, mais bon ça marche
    LedAnimK2000();
  }
}
