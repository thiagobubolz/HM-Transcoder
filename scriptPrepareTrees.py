
fout = open("DepthFinal.csv", "w")

#fout = open("traceResult" + nome + rate + ".csv", "w")
f1 = open("/home/thiago/Pesquisa/TrabalhoTCAS/HM-Trees/HM-16.4/trace2.csv", "r")
f2 = open("/home/thiago/Pesquisa/TrabalhoTCAS/HM-Trees/HM-16.4/traceMatlab.csv", "r")

while(True):
	#Lendo uma linha de cada arquivo
	linha1 = f1.readline()
	if not linha1: break
	linha2 = f2.readline()
	if not linha2: break

	#retirando o \n no final da linha
	off1 = linha1.split("\n")
	off2 = linha2.split("\n")

	#separando as linhas 
	aux1 = off1[0].split(",")
	aux2 = off2[0].split(",")

	soma1 = 0 
	soma2 = 0

	verifica1 = False
	verifica2 = False

	#Laco para verificar se existe alguma profundidade 1 ou 2 que desejam parar um nivel acima
	for i in range(len(aux1)):
		if aux1[i] == '1':
			if aux2[i] == '-1':
				verifica1 = True
		if aux1[i] == '2':
			if aux2[i] == '-1':
				verifica2 = True

	#Se existe alguma CU de tamanho 1 que quer parar um nivel acima entra no if
	if verifica1:
		#Laco para calcular com um TH se deve parar um nivel acima
		for j in range(len(aux1)):
			if aux1[j] == '1':
				soma1 = soma1 + int(aux2[j])*1024
			elif aux1 == '2'or aux1 == '3':
				soma1 = soma1 + int(aux2[j])*256
		#Caso TH esteja no limite para no limite 0
		if (soma1/4096) < (-0.5):
			fout.write("0")
		#Caso nao esteja no limite troca todos os valores -1 relacionados a profundidades 1 e modifica pra 0
		else:
			for k in range(len(aux1)):
				if aux1[k] == '1' and aux2[k] == '-1':
					aux2[k] = '0'
			#Verifica se tiveram profundidades 2 que deveriam terminar um nivel acima
			if verifica2:
				l=0
				#percorre a linha escrevendo as profundiades 1 no arquivo de saida
				while l < (len(aux1)):
					if aux1[l] == '1' and aux2[l] == '0': 
						fout.write(aux1[l])
						l = l + 1
					elif aux1[l] == '1' and aux2[l] == '1': 
						fout.write('2222')
						l = l + 1
					#Verifica o TH para saber se deve voltar um nivel
					elif aux1[l] == '2'or aux1[l] == '3':
							soma2 = soma2 + int(aux2[l])*256
							soma2 = soma2 + int(aux2[l+1])*256
							soma2 = soma2 + int(aux2[l+2])*256
							soma2 = soma2 + int(aux2[l+3])*256

							#Caso esteja dentro do TH escreve um nivel acima
							if (soma2/1024) < (-0.5):
								fout.write("1")
								l = l+4

							#Caso esteja fora do TH escreve o resto das profundiades no arquivo
							else :
								while l < l+4:
									if aux1[l] == '3' and aux2[l] == '-1':
										fout.write('2')
										l = l+1
									if aux2[l] == '0' or aux2 == '-1':
										fout.write(aux1[l])
										l = l+1
									elif aux2[l] == '1':
										fout.write(int(aux1[l]) + 1)
										l = l+1
			#Caso nao tenha mais nenhuma profundidade 2 ou 1 que deveria parar um nivel antes entra aqui
			else: 
				print aux1
				print aux2
				l=0
				while l < (len(aux1)):
					if aux1[l] == '1' and aux2[l] == '0': 
						fout.write(aux1[l])
						l = l + 1
					elif aux1[l] == '1' and aux2[l] == '1': 
						fout.write('2222')
						l = l + 1
					elif aux1[l] == '2'or aux1[l] == '3':
						parada = l+4
						while l < parada:
							if aux1[l] == '3' or aux2 == '-1':
								fout.write('2')
								l = l+1
							elif aux2[l] == '0' or aux2 == '-1':
								fout.write(aux1[l])
								l = l+1
							elif aux2[l] == '1':
								fout.write(str(int(aux1[l]) + 1))
								l = l+1

	# Se existem apenas profundidades 2 que devem ficar um nivel antes
	elif verifica2:
		l=0
		while l < (len(aux1)):
			if aux1[l] == '1' and aux2[l] == '0': 
				fout.write(aux1[l])
				l = l + 1
			elif aux1[l] == '1' and aux2[l] == '1': 
				fout.write('2222')
				l = l + 1
			elif aux1[l] == '2'or aux1[l] == '3':
				soma2 = soma2 + int(aux2[l])*256
				soma2 = soma2 + int(aux2[l+1])*256
				soma2 = soma2 + int(aux2[l+2])*256
				soma2 = soma2 + int(aux2[l+3])*256

				if ((soma2/1024) < (-0.5)):
					fout.write('1')
					l = l+4
				else :
					parada = l+4
					while l < parada:
						if aux1[l] == '3' and aux2[l] == '-1':
							fout.write('2')
							l = l+1
						elif aux2[l] == '0' or aux2 == '-1':
							fout.write(aux1[l])
							l = l+1
						elif aux2[l] == '1':
							fout.write(str(int(aux1[l]) + 1))
							l = l+1
	elif aux1[0] == '0' :
		if aux1[0] == '0' and aux2[0] == '1':
			fout.write('1111')
		else:
			fout.write('0') 
	else: 
		#1,3,2,3,2,1,1
		#1,0,1,0,1,1,1

		l=0
		while l < (len(aux1)):
			if aux1[l] == '1' and aux2[l] == '0': 
				fout.write(aux1[l])
				l = l + 1
			elif aux1[l] == '1' and aux2[l] == '1': 
				fout.write('2222')
				l = l + 1
			elif aux1[l] == '2'or aux1[l] == '3':
				parada = l+4
				while l < parada:
					if aux1[l] == '3' and aux2[l] == '0':
						fout.write('3')
						l= l+1
					elif aux1[l] == '3' and aux2[l] == '-1':
						fout.write('2')
						l= l+1
					elif aux1[l] == '2' and aux2[l] == '0':
						fout.write('2')
						l = l+1
					elif aux1[l] == '2' and  aux2[l] == '1':
						fout.write('3')
						l = l+1	

	fout.write("\n")						