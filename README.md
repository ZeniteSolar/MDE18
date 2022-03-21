#MDE18

_por Cesar Dias Parente_

Com a escolha correta do motor, engrenagens e correias, demos inicio aos módulos de controle da direção para obtermos uma melhor resposta do sistema ao todo. Inicialmente escolhemos como controlar o motor DC 12v, em estudos realizados optamos por utilizar de um circuito eletrônico que possui uma ponte H completa baseado em dois chips BTS7960 em conjunto , obtendo assim o controle do motor por PWM, desta forma teremos uma melhor escolhe de potencia aplicada, e reversão do motor.

A escolha do componente que faz o controle do modulo de ponte H, foi o microcontrolador Atmega 328p, que é facilmente encontrado em mercado, usando da plataforma do Arduino nano pelo seu tamanho reduzido. Para obtermos o melhor resultado de controle, foi estudado SERVOMOTORES, onde também utilizam de um motor DC, um microcontrolador, e um sensor de ângulo para verificação de posição, com isso criamos uma malha fechada de controle.

Com o conjunto de controle do leme já finalizado, partimos para obtenção de sinal de ângulo da direção, onde usamos um potenciômetro pegando o ângulo diretamente no eixo do bloco da direção, mandando o sinal para um microcontrolador Atmega 328p, em uma porta com conversor A/D. Para um sinal mais efetivo e diminuição de ruídos e variações indesejadas da direção, foi realizado na programação do microcontrolador uma media, podendo ser ajustada pelo firmware.

Foram utilizados dois microcontroladores independentes, pois o barco possui uma distancia entre Proa e Popa de em media 6 metros, desta forma os microcontroladores ficam próximos dos seus componentes mais importantes, com isso diminuindo a chance de interferências eletromagnéticas indesejadas. Foi estudada a melhor forma de fazer a comunicação entre eles, notando que o protótipo náutico já possui uma malha de rede de comunicação CAN, utilizamos por padrão a mesma comunicação, porém uma rede totalmente independente da já existente, pois foi escolhida a forma de REAL TIME para a direção, onde o modulo frontal estará sempre mandando a informação do ângulo no exato momento, desta forma ocupando 100% da comunicação, garantindo o melhor funcionamento da direção.

Em vista do projeto, notamos certa necessidade em ter um funcionamento ininterrupto, diminuindo possíveis problemas devido ao ambiente extremo que estará os componentes eletrônicos. Em primeira partida verificamos a necessidade de um cabo adequado para a comunicação CAN, onde foi utilizado um cabo UTP CAT-5E em alumínio e flexível, desta forma a oxidação e a vibração que poderia criar uma perca total de comunicação por rompimento de um fio de comunicação ou alimentação será diminuída. Nas conexões, utilizamos conectores com maior resistência a agua. Os sensores de ângulos possuindo uma maior resistência a água, onde o da proa foi encapsulado com termoretratil, aumentando a rigidez dos fios na conexão soldada do potenciômetro e dificultando o contato direto com a agua. Os circuitos eletrônicos foram inseridos em caixas herméticas e mergulhados em resina de silicone com cura ambiente, onde únicas partes que ficaram expostas são os cabos assim eliminando a chance de contato com a agua do mar, e aumentando a resistência mecânica dos circuitos, diminuindo as chances de quebra de solda, oxidação das placas, falhas nas conexões entre placas e circuitos externos.
