/**
 * TP4
 * 
 */

#include "stm32f10x.h"

// Fonction de d�lai logiciel (t�che longue qui sera interrompue)
void Delay( volatile uint32_t count) {
    for(uint32_t i = 0; i < count; i++) {
        // Cette boucle simule une t�che longue
        // Elle sera interrompue si un �v�nement externe survient
    }
}

// Initialisation des GPIO
void GPIO_Config(void) {
    // 1. Activer les horloges pour GPIOA, GPIOB et GPIOC
    RCC->APB2ENR |= (1 << 2);  // IOPAEN
    RCC->APB2ENR |= (1 << 3);  // IOPBEN
    RCC->APB2ENR |= (1 << 4);  // IOPCEN
    
    // 2. Configurer PA5 (LED1 - LED verte int�gr�e) en sortie push-pull
	  //  LED de la tache de fond (qui sera interrompu)
    GPIOA->CRL &= ~(0xF << 20);  // Effacer la configuration existante
    GPIOA->CRL |= (0x3 << 20);   // Output 50MHz
    
    // 3. Configurer PB3 (LED2) en sortie push-pull
	 // //  LED gerer par interruption
    GPIOB->CRL &= ~(0xF << 12);  // Effacer la configuration existante
    GPIOB->CRL |= (0x3 << 12);   // Output 50MHz
	
	    
    // 4. Configurer PB4 (LED2) en sortie push-pull
	 // LED g�r�e par interruption (PB4)
		GPIOB->CRL &= ~(0xF << 16);  // Effacer la configuration existante
		GPIOB->CRL |= (0x3 << 16);   // Output 50MHz
    
    // 5. Configurer PC13 (Bouton int�gr�) en entr�e avec pull-up
	  //  source de interruption
    GPIOC->CRH &= ~(0xF << 20);  // Effacer la configuration existante
    GPIOC->CRH |= (0x8 << 20);   // Input avec pull-up/down
    GPIOC->ODR |= (1 << 13);     // Activer la r�sistance de pull-up
	
	// 6. Configurer PC3 en entr�e avec pull-up
		// Source d'interruption
		GPIOC->CRL &= ~(0xF << 12);  // Effacer la configuration existante (4 bits � la position 12 pour PC3)
		GPIOC->CRL |= (0x8 << 12);   // Input avec pull-up/down
		GPIOC->ODR |= (1 << 3);      // Activer la r�sistance de pull-up
		
}

// Configuration de l'interruption externe EXTI13
void EXTI13_Config(void) {
    // 1. Activer l'horloge AFIO
    RCC->APB2ENR |= (1 << 0);    // AFIOEN
    
    // 2. Connecter PC13 � EXTI13
    AFIO->EXTICR[3] &= ~(0xF << 4);  // Effacer la configuration existante
    AFIO->EXTICR[3] |= (2 << 4);     // S�lectionner PC13 (2 = GPIOC)
    
    // 3. Configurer EXTI13
    EXTI->IMR |= (1 << 13);      // Activer l'interruption sur la ligne EXTI13
    EXTI->FTSR |= (1 << 13);     // D�clencher sur front descendant (bouton press�)
    
    // 4. Configurer et activer l'interruption dans le NVIC
    NVIC_SetPriority(EXTI15_10_IRQn, 2);  // EXTI13 est dans le groupe EXTI15_10
	  NVIC_EnableIRQ(EXTI15_10_IRQn);       // Activer l'interruption dans le NVIC
}


// Configuration de l'interruption externe EXTI13
void EXTI3_Config(void) {
 // 1. Activer l'horloge AFIO
	RCC->APB2ENR |= (1 << 0);    // AFIOEN

	// 2. Connecter PC3 � EXTI3
	AFIO->EXTICR[0] &= ~(0xF << 12);  // Effacer la configuration existante
	AFIO->EXTICR[0] |= (2 << 12);     // S�lectionner PC3 (2 = GPIOC)

	// 3. Configurer EXTI3
	EXTI->IMR |= (1 << 3);      // Activer l'interruption sur la ligne EXTI3
	EXTI->FTSR |= (1 << 3);     // D�clencher sur front descendant

	// 4. Configurer et activer l'interruption dans le NVIC
	NVIC_SetPriority(EXTI3_IRQn, 1);  // EXTI3 a sa propre ligne d'interruption
	NVIC_EnableIRQ(EXTI3_IRQn);       // Activer l'interruption dans le NVIC
}



// Gestionnaire d'interruption pour EXTI15_10 (dont fait partie EXTI13)
void EXTI15_10_IRQHandler(void) {
    // V�rifier que l'interruption provient bien d'EXTI13
    if (EXTI->PR & (1 << 13)) {
         // Effacer le drapeau d'interruption
        EXTI->PR = (1 << 13); 
        
        // Inverser l'�tat de LED2
        GPIOB->ODR ^=(1<<4);
      
    }
}

// Gestionnaire d'interruption pour EXTI3 (dont fait partie EXTI3)
void EXTI3_IRQHandler(void) {
    // V�rifier que l'interruption provient bien d'EXTI13
    if (EXTI->PR & (1 << 3)) {
         // Effacer le drapeau d'interruption
        EXTI->PR = (1 << 3); 
        
        // Inverser l'�tat de LED3
        GPIOB->ODR ^=(1<<3);
      
    }
}


int main(void) {
    // Initialiser les GPIO et l'interruption externe
    GPIO_Config();
    EXTI13_Config();
	  EXTI3_Config();
    
    // Initialiser les LEDs � l'�tat �teint
    GPIOA->BRR = (1 << 5);    // PA5 = 0 (LED1 �teinte)
    GPIOB->BRR = (1 << 3);    // PB3 = 0 (LED2 �teinte)
    
    // Boucle principale
    while (1) {
        // Inverser l'�tat de LED1
        GPIOA->ODR ^= (1 << 5);
        
        // D�lai logiciel long (t�che de fond)
        // Cette t�che sera interrompue par l'appui sur le bouton
        Delay(5000000);  // Environ 1-2 secondes selon la fr�quence du MCU
    }
}