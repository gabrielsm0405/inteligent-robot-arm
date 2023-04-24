# inteligent-robot-arm

Indentificação

O arquivo automation_project_identification.py gera um modelo de Arvore de decisões
otimizado por algoritmo genético, plotanto e printando os resultados. O modelo resultante 
é armazenado no arquivo identification.joblib.
Para importar o modelo através do arquivo basta usar a biblioteca joblib:

from joblib import load
model = load('identification.joblib')