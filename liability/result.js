import Web3 from 'web3';
import { readFileSync } from "fs";
//import { off } from 'process';

const web3 = new Web3("https://proxy.devnet.neonlabs.org/solana")

let args = process.argv.slice(2)
let config = readFileSync(`${args[0]}/config/config`)
let json_config = JSON.parse(config)
const xrt_address = json_config["xrt_contract"]
const liability_address = json_config["liability_contract"]

let result_ipfs = args[1]
// console.log(result_ipfs)


let robot_account = json_config["robot_address"]
await web3.eth.accounts.wallet.add(json_config["robot_private_key"])
web3.eth.defaultAccount = robot_account

async function send_result(web3, result_ipfs) {
    let abi = readFileSync(`${args[0]}/liability/abi/Liability.json`)
    let json_abi = JSON.parse(abi)
    
    let liability = await new web3.eth.Contract(json_abi, liability_address)
    const robot_private_key = json_config["robot_private_key"]
    // const result = web3.utils.randomHex(34);
    const hash = web3.utils.soliditySha3(
        {t: 'address',   v: liability_address},
        {t: 'bytes',     v: web3.utils.toHex(result_ipfs)},
        {t: 'bool',      v: true}
    );
    // console.log(result)
    // console.log(hash)
    const signature = await web3.eth.accounts.sign(hash, robot_private_key); 
    // console.log(signature)

    let tx = await liability.methods.finalize(web3.utils.toHex(result_ipfs), true, signature.signature).send({from: robot_account, gas: 1000000000})
    console.log(tx.transactionHash)
}



await send_result(web3, result_ipfs)
