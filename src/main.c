#include <motors.h>
#include <ultrassonic.h>

int main(void)
{   
    printk("Iniciando robô no modo simples...\n");

    //Inicializa o hardware
    ultrassonic_init();
    motors_init();

    //Tempo para a eletrônica estabilizar
    k_msleep(500);

    //O carrinho já começa andando em frente
    printk("Acelerando...\n");
    go(); 

    //Loop contínuo
    for(;;) {
        // Lê o sensor ultrassônico
        int distancia = distance_cm();
        
        // Verifica se há um obstáculo real na frente
        if (distancia > 6 && distancia < 20) {
            //printk("Obstáculo a %d cm! Desviando...\n", distancia);
            
            // Passo A: Freia imediatamente
            stop();
            k_msleep(500); // Espera o chassi parar totalmente
            
            // Passo B: Desvia para direita
            right();
            
            // Passo C: Estabiliza após a curva
            stop();
            k_msleep(200); 
            
            // Passo D: Retoma a viagem em frente
            //printk("Caminho livre. Acelerando...\n");
            go(); 
        }

        //Tempo para desafogar o processador
        k_msleep(50); 
    }

    return 0;
}