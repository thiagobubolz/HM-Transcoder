#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char const *argv[]){

	int perc64x64 = 0, perc64x32 = 0, perc64x16 = 0, perc64x8 = 0, perc32x64 = 0, perc32x32 = 0, perc32x16 = 0, perc32x8 = 0, perc16x64 = 0, perc16x32 = 0, perc16x16 = 0, perc16x8 = 0, perc8x64 = 0, perc8x32 = 0, perc8x16 = 0, perc8x8 = 0;

	int seq1[256];
	int seq2[256];

	ifstream arquivo1, arquivo2; 
	ofstream arquivo3;
	arquivo1.open("KristenAndSara_100.csv", ios::in);
	arquivo2.open("KristenAndSara_20.csv", ios::in);
	arquivo3.open("KristenAndSara_20_mod_result.csv", ios::out);

	string linha1, linha2, lixo1, lixo2;

	while(getline(arquivo1,lixo1) && getline(arquivo2, lixo2)){

		int k=0;

		getline(arquivo1, linha1);
		getline(arquivo2, linha2);

		for (int i = 0; i < 514; ++i){
			if (linha1[i] == '0' || linha1[i] == '1' || linha1[i] == '2' || linha1[i] == '3'){
				seq1[k] = linha1[i] - '0';
			}
			if (linha2[i] == '0' || linha2[i] == '1' || linha2[i] == '2' || linha2[i] == '3'){
				seq2[k] = linha2[i] - '0';
				k++;
			}
		}

		for (int j = 0; j < 256; ++j){
			if(seq1[j] == 0 && seq2[j] == 0){
				perc64x64++;
			}
			if(seq1[j] == 0 && seq2[j] == 1){
				perc64x32++;
			}
			if(seq1[j] == 0 && seq2[j] == 2){
				perc64x16++;
			}
			if(seq1[j] == 0 && seq2[j] == 3){
				perc64x8++;

			}
			if(seq1[j] == 1 && seq2[j] == 0){
				perc32x64++;
			}
			if(seq1[j] == 1 && seq2[j] == 1){
				perc32x32++;
			}
			if(seq1[j] == 1 && seq2[j] == 2){
				perc32x16++;
			}
			if(seq1[j] == 1 && seq2[j] == 3){
				perc32x8++;
			}
			if(seq1[j] == 2 && seq2[j] == 0){
				perc16x64++;
			}
			if(seq1[j] == 2 && seq2[j] == 1){
				perc16x32++;
			}
			if(seq1[j] == 2 && seq2[j] == 2){
				perc16x16++;
			}
			if(seq1[j] == 2 && seq2[j] == 3){
				perc16x8++;
			}
			if(seq1[j] == 3 && seq2[j] == 0){
				perc8x64++;
			}
			if(seq1[j] == 3 && seq2[j] == 1){
				perc8x32++;
			}
			if(seq1[j] == 3 && seq2[j] == 2){
				perc8x16++;
			}
			if(seq1[j] == 3 && seq2[j] == 3){
				perc8x8++;
			}
		}

		getline(arquivo1, linha1);
		getline(arquivo2, linha2);
	}

	int somatorio64 = perc64x64 + perc64x32 + perc64x16 + perc64x8;
	int somatorio32 = perc32x64 + perc32x32 + perc32x16 + perc32x8;
	int somatorio16 = perc16x64 + perc16x32 + perc16x16 + perc16x8;
	int somatorio8 = perc8x64 + perc8x32 + perc8x16 + perc8x8;
	int somatorio_total = somatorio64 + somatorio32 + somatorio16 + somatorio8;

	arquivo3 << (float) perc64x64*100/somatorio64 << " " << (float) perc64x32*100/somatorio64 << " " << (float) perc64x16*100/somatorio64 << " " << (float) perc64x8*100/somatorio64 << endl;
	arquivo3 << (float) perc32x64*100/somatorio32 << " " << (float) perc32x32*100/somatorio32 << " " << (float) perc32x16*100/somatorio32 << " " << (float) perc32x8*100/somatorio32 << endl;
	arquivo3 << (float) perc16x64*100/somatorio16 << " " << (float) perc16x32*100/somatorio16 << " " << (float) perc16x16*100/somatorio16 << " " << (float) perc16x8*100/somatorio16 << endl;
	arquivo3 << (float) perc8x64*100/somatorio8 << " " << (float) perc8x32*100/somatorio8 << " " << (float) perc8x16*100/somatorio8 << " " << (float) perc8x8*100/somatorio8 << endl;

	arquivo3 << (float) somatorio64*100/somatorio_total << endl;
	arquivo3 << (float) somatorio32*100/somatorio_total << endl;
	arquivo3 << (float) somatorio16*100/somatorio_total << endl;
	arquivo3 << (float) somatorio8*100/somatorio_total << endl;

	arquivo1.close();
	arquivo2.close();
	arquivo3.close();

	return 0;
}